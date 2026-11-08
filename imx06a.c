// SPDX-License-Identifier: GPL-2.0
/*
 * V4L2 driver for the Sony IMX06A-AJ1R-J (LYT-900 silicon): a 50.3 Mpixel
 * Type 1/0.98 stacked BSI sensor with a Quad-Bayer colour filter, on-chip QBC
 * remosaic and a CCS-style register map, on four CSI-2 D-PHY lanes.
 *
 * Modes (imx06a_modes[]): full resolution 8192x6144 RAW10/RAW12 (4:3, 16:9),
 * 4x2-binned 4096x3072 RAW10 (4:3, 16:9, 4K UHD and 1080p crops), and two
 * single-frame HDR families, DCG-HDR (RAW12/RAW14) and QBC-HDR (RAW12).
 *
 * The sensor's D-PHY runs up to 4.5 Gbps/lane; this driver synthesises the
 * clock tree for any lane rate from 300 to 2500 Mbps (see "Clocking"). The
 * overlay default is 1752 Mbps; 2112 Mbps is the fastest the Raspberry Pi 5
 * receiver takes error-free. Optional extras: the phase-detect stream on a
 * second pad and frame sync outputs on the GPO / TESTOUT pins.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

#include "imx06a_tables.h"
#include "imx06a_tables_4x4.h"

/* --------------------------------------------------------------------------
 * Registers
 * --------------------------------------------------------------------------
 */
#define IMX06A_REG_FRM_CNT		CCI_REG8(0x0005)
#define   IMX06A_FRM_CNT_STANDBY	0xff
#define IMX06A_REG_CHIP_ID		CCI_REG16(0x0016)
#define   IMX06A_CHIP_ID		0xa18a
#define IMX06A_REG_REVISION		CCI_REG8(0x0018)

#define IMX06A_REG_MODE_SELECT		CCI_REG8(0x0100)
#define   IMX06A_MODE_STANDBY		0x00
#define   IMX06A_MODE_STREAMING		0x01
#define IMX06A_REG_ORIENTATION		CCI_REG8(0x0101)
#define IMX06A_REG_GROUP_HOLD		CCI_REG8(0x0104)
#define IMX06A_REG_FAST_STANDBY		CCI_REG8(0x0106)
#define IMX06A_REG_EXCK_FREQ		CCI_REG16(0x0136)

#define IMX06A_REG_EXPOSURE		CCI_REG16(0x0202)
#define   IMX06A_EXPOSURE_MARGIN	64	/* CIT_MARGIN (0x68C2), read-only */
#define   IMX06A_EXPOSURE_DEFAULT	1000	/* lines above the mode minimum */
#define IMX06A_REG_ANALOG_GAIN		CCI_REG16(0x0204)
#define   IMX06A_ANA_GAIN_MIN		0
#define   IMX06A_ANA_GAIN_SCALE		16384	/* gain = 16384 / (16384 - code) */
#define IMX06A_REG_DIGITAL_GAIN		CCI_REG16(0x020e)
#define IMX06A_REG_ST_DIGITAL_GAIN	CCI_REG16(0x0218)	/* = 0x020E in DCG-HDR */
#define   IMX06A_DGTL_GAIN_MIN		0x0100
#define   IMX06A_DGTL_GAIN_MAX		0x0fff
#define   IMX06A_DGTL_GAIN_DEFAULT	0x0100

/* PLL */
#define IMX06A_REG_IVT_PXCK_DIV		CCI_REG8(0x0301)
#define   IMX06A_IVT_PXCK_DIV		10
#define IMX06A_REG_IVT_SYCK_DIV		CCI_REG8(0x0303)
#define   IMX06A_IVT_SYCK_DIV		2
#define IMX06A_REG_IVT_PREPLLCK_DIV	CCI_REG8(0x0305)
#define IMX06A_REG_IVT_PLL_MPY		CCI_REG16(0x0306)
#define IMX06A_REG_IOP_SYCK_DIV		CCI_REG8(0x030b)
#define IMX06A_REG_IOP_PREPLLCK_DIV	CCI_REG8(0x030d)
#define IMX06A_REG_IOP_PLL_MPY		CCI_REG16(0x030e)

/* Frame and line timing */
#define IMX06A_REG_FRM_LENGTH		CCI_REG16(0x0340)
#define   IMX06A_FRM_LENGTH_MAX		65532U	/* lowest common max (Table 4-5) */
#define IMX06A_REG_LINE_LENGTH_PCK	CCI_REG16(0x0342)
#define IMX06A_REG_MC_FRM_LP_EN		CCI_REG8(0x306d)
#define IMX06A_REG_LINE_LENGTH_INCK	CCI_REG16(0x3740)
#define   IMX06A_LLINCK_MAX		65520U
/*
 * Long exposure (SRM 11.3): CIT_LSHIFT = n multiplies COARSE_INTEG_TIME and
 * FRM_LENGTH_LINES by 2^n. VBLANK and EXPOSURE are exposed in real lines up to
 * 2^9 times the register range; the driver programs value >> n.
 */
#define IMX06A_REG_CIT_LSHIFT		CCI_REG8(0x3150)
#define   IMX06A_LONG_EXP_SHIFT_MAX	9

/* Geometry (SRM 4.3 / 5.2.5) */
#define IMX06A_REG_X_ADD_STA		CCI_REG16(0x0344)
#define IMX06A_REG_Y_ADD_STA		CCI_REG16(0x0346)
#define IMX06A_REG_X_ADD_END		CCI_REG16(0x0348)
#define IMX06A_REG_Y_ADD_END		CCI_REG16(0x034a)
#define IMX06A_REG_X_OUT_SIZE		CCI_REG16(0x034c)
#define IMX06A_REG_Y_OUT_SIZE		CCI_REG16(0x034e)
#define IMX06A_REG_DIG_CROP_X_OFFSET	CCI_REG16(0x0408)
#define IMX06A_REG_DIG_CROP_Y_OFFSET	CCI_REG16(0x040a)
#define IMX06A_REG_DIG_CROP_WIDTH	CCI_REG16(0x040c)
#define IMX06A_REG_DIG_CROP_HEIGHT	CCI_REG16(0x040e)
#define IMX06A_NATIVE_WIDTH		8192U
#define IMX06A_NATIVE_HEIGHT		6144U

/*
 * Output black level, exposed as V4L2_CID_BRIGHTNESS (as in the imx585 driver)
 * in 12-bit LSBs in every mode, i.e. x16 in 16-bit units; default 256 = 4096.
 *  - MANUAL_DATA_PEDESTAL_VALUE (SRM 5.6, Table 5-58), u10.2: the level the
 *    optical black clamp holds. Normal modes output it as is (one unit = one
 *    12-bit LSB); the HDR merge outputs it as value / 4 LSB in its own depth.
 *  - PEDESTAL_OFFSET (SRM 9.4, HDR only, 10 bit): added after the merge in
 *    14-bit LSBs (offset / 4 at 12 bit).
 * imx06a_write_black_level() splits the target between the two in HDR.
 */
#define IMX06A_REG_DATA_PEDESTAL	CCI_REG16(0x302c)
#define   IMX06A_DATA_PEDESTAL_MAX	0x0fff
#define   IMX06A_BLACK_LEVEL_DEFAULT	0x0100
#define IMX06A_REG_PEDESTAL_OFFSET_EN	CCI_REG8(0x3214)
#define IMX06A_REG_PEDESTAL_OFFSET	CCI_REG16(0x32f0)
#define   IMX06A_PEDESTAL_OFFSET_MAX	0x03ff

/* D-PHY skew calibration bursts */
#define IMX06A_REG_SCAL_PERIOD_EN	CCI_REG8(0x0830)
#define IMX06A_REG_SCAL_INIT_EN		CCI_REG8(0x0832)

#define IMX06A_REG_TEST_PATTERN		CCI_REG16(0x0600)
#define IMX06A_REG_TEST_PATTERN_COLOUR(n) CCI_REG16(0x0602 + 2 * (n)) /* R, Gr, B, Gb */
#define   IMX06A_TEST_PATTERN_COLOUR_MAX 0x0fff

/* White-balance hint for the on-chip QBC remosaic (8.8 fixed point) */
#define IMX06A_REG_ABS_GAIN_GR		CCI_REG16(0x0b8e)
#define IMX06A_REG_ABS_GAIN_R		CCI_REG16(0x0b90)
#define IMX06A_REG_ABS_GAIN_B		CCI_REG16(0x0b92)
#define IMX06A_REG_ABS_GAIN_GB		CCI_REG16(0x0b94)
#define   IMX06A_WB_GAIN_MIN		0x0001
#define   IMX06A_WB_GAIN_MAX		0x0fff
#define   IMX06A_WB_GAIN_UNITY		0x0100

/*
 * Sync outputs (SRM 11.5 monitor terminals, 11.6 long FSTROBE) on the GPO (L1,
 * 4 mA) and TESTOUT (C1, 2 mA) pins, 1.8 V CMOS, each with its own selector.
 * Set in SW standby, so they are applied at stream start.
 */
#define IMX06A_REG_FSTROBE_MODE		CCI_REG8(0x3228)
#define   IMX06A_FSTROBE_LONG		2
#define IMX06A_REG_MON_SEL		CCI_REG16(0x4112)
#define   IMX06A_MON_SEL_DEFAULT	0x3ff	/* monitors off; 0 = on */
#define IMX06A_REG_GPIO_SEL		CCI_REG16(0x4048)
#define IMX06A_REG_MNTTEST1_SEL		CCI_REG16(0x6848)
#define IMX06A_REG_MNTTEST2_SEL		CCI_REG16(0x684a)
#define   IMX06A_MNTTEST_HSYNC		0x015
#define   IMX06A_MNTTEST_VSYNC		0x016
#define IMX06A_REG_MON_GPO		CCI_REG8(0x4102)
#define IMX06A_REG_MON_TESTOUT		CCI_REG8(0x4103)
#define   IMX06A_MON_LONG_FSTROBE	0x2c
#define   IMX06A_MON_MNTTEST1		0xff
#define   IMX06A_MON_MNTTEST2		0xfe
#define IMX06A_REG_IO_CTRL_GPO		CCI_REG8(0x402e)
#define IMX06A_REG_IO_CTRL_TESTOUT	CCI_REG8(0x402f)
#define IMX06A_REG_GPO_CTRL_SEL		CCI_REG8(0x3037)
#define IMX06A_REG_TESTOUT_CTRL_SEL	CCI_REG8(0x303a)

/*
 * Phase-detect (PD) output. Every pixel is a dual photodiode. With PHASE_PIX
 * (0x3104, undocumented) the sensor sends one PD
 * packet per 4 image lines on VC0 with the MIPI user-defined data type 0x30:
 * interleaved left/right photodiode values (L + R = the image), always RAW10.
 * Full-res AD12 instead sends a sparse PD pattern on the same VC/DT, enabled by
 * PDAF_TYPE2 (0x3103); see imx06a_pd_geom(). The stream is exposed on a second
 * source pad when the "pdaf" option is set at probe.
 */
#define IMX06A_REG_PDAF_TYPE2		CCI_REG8(0x3103)
#define IMX06A_REG_PHASE_PIX		CCI_REG8(0x3104)
#define IMX06A_PD_PIXELS		4096U
#define IMX06A_PD_BPP			10U
#define IMX06A_PD_LINE_GROUP		4U
#define IMX06A_PD_DT			MIPI_CSI2_DT_USER_DEFINED(0)
#define IMX06A_PD_SPARSE_PX		1016U
#define IMX06A_PD_SPARSE_Y0		64U	/* array rows carrying sparse PD */
#define IMX06A_PD_SPARSE_Y1		6080U
/*
 * The PD pad is advertised at the RP1 CFE's default embedded size in every
 * mode, not at the true PD size: the sensor-to-CSI-2 link is immutable and
 * validated at every stream start, and libcamera never sets the CSI-2 metadata
 * pads, so a per-mode size would break every libcamera stream. The receiver
 * stores whatever DT 0x30 data arrives, up to the buffer size its consumer sets.
 */
#define IMX06A_PD_PAD_WIDTH		16384U

#define IMX06A_CSI2_VC_MASK		0x03
#define IMX06A_CSI2_DT_MASK		0x3f

/* Vendor controls */
#define IMX06A_CID_HDR_MODE		(V4L2_CID_USER_BASE + 0x2100)
#define IMX06A_CID_GPO_OUT		(V4L2_CID_USER_BASE + 0x2101)
#define IMX06A_CID_TESTOUT_OUT		(V4L2_CID_USER_BASE + 0x2102)

/*
 * Power-on timing (datasheet 7-2 / SRM Table 6-1): 1.6 ms to the first CCI
 * access, 6.6 ms before the streaming command (OTP load).
 */
#define IMX06A_POWER_ON_DELAY_US	10000
#define IMX06A_AUTOSUSPEND_MS		1000

/* --------------------------------------------------------------------------
 * Clocking
 * --------------------------------------------------------------------------
 *
 * Two PLLs from INCK (12..27 MHz):
 *   IVTCK = INCK / IVT_PREPLLCK_DIV * IVT_PLL_MPY     1050..2196 MHz (44..183)
 *   IVTPXCK = IVTCK / (IVT_SYCK_DIV * IVT_PXCK_DIV)    (2 x 10)
 *   pixel rate (LINE_LENGTH_PCK units) = IVTPXCK * 32 = 1.6 * IVTCK
 *   IOPCK = INCK / IOP_PREPLLCK_DIV * IOP_PLL_MPY     2150..4500 MHz (90..4500)
 *   lane rate = IOPCK / IOP_SYCK_DIV (1, 2, 4, 8)
 *
 * LINE_LENGTH_PCK is not free: SRM Table 4-6 lists the legal values per mode
 * (others put the internal H sync on interference, causing FPN). Every mode
 * therefore runs at its shortest legal LLPCK and the fastest IVTCK, and the
 * line PERIOD is set separately with
 * LINE_LENGTH_INCK, which governs frame timing once MC_FRM_LP_EN = 1:
 *   line time = LINE_LENGTH_INCK / INCK            (SRM 5.3 / 5.4.2)
 * LLINCK only adds idle time at the end of the line; FPN and noise do not change.
 *
 * V4L2 timing base: PIXEL_RATE = INCK x 64 and HBLANK in steps of 64, so one
 * HBLANK step is one INCK cycle. HBLANK's minimum is the larger of
 *   - the readout floor:  ceil(LLPCK * INCK / (1.6 * IVTCK))  (SRM 4.5.1), and
 *   - the CSI-2 link floor: one output line (plus PD packets when enabled) over
 *     4 lanes at the selected rate, with per-line D-PHY overhead.
 * Receiver limits (e.g. the PiSP front-end pixel rate) are left to libcamera,
 * which lengthens HBLANK itself.
 */
#define IMX06A_INCK_MIN_HZ		12000000U
#define IMX06A_INCK_MAX_HZ		27000000U
#define IMX06A_IVTCK_MIN_HZ		1050000000ULL
#define IMX06A_IVTCK_MAX_HZ		2196000000ULL
#define IMX06A_IVT_MPY_MIN		44U
#define IMX06A_IVT_MPY_MAX		183U
#define IMX06A_IOPCK_MIN_HZ		2150000000ULL
#define IMX06A_IOPCK_MAX_HZ		4500000000ULL
#define IMX06A_IOP_MPY_MIN		90U
#define IMX06A_IOP_MPY_MAX		4500U
#define IMX06A_LINK_MBPS_MIN		300U	/* D-PHY timing range */
#define IMX06A_LINK_MBPS_MAX		2500U
#define IMX06A_LINK_MBPS_MAX_CLEAN	2112U	/* fastest error-free on the RP1 */
#define IMX06A_PIX_PER_INCK		64U	/* PIXEL_RATE = INCK * 64 */
/*
 * Per-line D-PHY overhead (LP<->HS transitions, packet header and footer) on
 * top of the payload. A shorter line keeps the frame rate, but the sensor drops
 * the last rows of every frame. Measured ~405 ns + 453 UI; rounded up for margin.
 */
#define IMX06A_LINE_OH_NS		500U
#define IMX06A_LINE_OH_UI		500U

/* --------------------------------------------------------------------------
 * Modes
 * --------------------------------------------------------------------------
 */

/*
 * HDR families (SRM 9 / 10), both single-frame sensor-fused outputs, so they are
 * ordinary linear Bayer streams to the receiver and to libcamera:
 *  - DCG-HDR: one exposure read at two conversion gains and merged; ratio 4:1
 *    gives RAW12, 16:1 RAW14. 4x2 binned readout only.
 *  - QBC-HDR: long and short exposures interleaved in each Quad-Bayer cell,
 *    merged on chip to RAW12. Short = long / 4. Uses the host WB gains.
 * The family is chosen with the "HDR Mode" control (or WIDE_DYNAMIC_RANGE,
 * 1 = DCG-HDR) before the format is negotiated and decides which mode list the
 * pad advertises.
 */
enum imx06a_hdr {
	IMX06A_HDR_OFF,
	IMX06A_HDR_DCG,
	IMX06A_HDR_QBC,
};

/* COARSE_INTEG_TIME rules (SRM Tables 5-25 and 10-4). */
enum imx06a_cit_rule {
	IMX06A_CIT_FULL,	/* min 6, step 2, prohibited 8..28 step 4 */
	IMX06A_CIT_BIN,		/* min 4, step 4, prohibited 8, 16 (binned, DCG-HDR) */
	IMX06A_CIT_QBC,		/* long CIT; short = CIT/4 must be >= 2, even, not 4/8 */
};

struct imx06a_mode {
	const char *name;
	u32 width;		/* output size */
	u32 height;
	u32 code;		/* RGGB base code */
	u32 llpck;		/* shortest legal LINE_LENGTH_PCK (Table 4-6) */
	u32 ro_llpck;		/* readout floor in LLPCK units if longer (0 = llpck) */
	u32 fll_min;		/* minimum frame length */
	u32 fll_step;		/* Table 4-5 */
	enum imx06a_cit_rule cit_rule;
	u32 exp_min;
	u32 exp_step;
	/*
	 * Analogue gain, always gain = 16384 / (16384 - code). In DCG-HDR the
	 * register holds the high-side gain (at least 12 dB at 4:1, 24 dB at 16:1)
	 * while the merged output is referenced to the low side (LSG = HSG / ratio),
	 * so the control exposes the LSG and imx06a_hw_gain() converts. gain_step
	 * keeps the register value a multiple of 4.
	 */
	u32 gain_max;
	u32 gain_step;
	u32 hdr_ratio;		/* DCG-HDR HSG:LSG ratio (4 or 16), 0 otherwise */
	enum imx06a_hdr hdr;
	u32 bin;		/* 1 = full resolution, 2 = 4x2 / diagonal binning */
	u32 y_start;		/* analogue window start row (multiple of 32) */
	u32 rows;		/* analogue window rows (multiple of 32) */
	u32 dig_x;		/* digital crop offset, post-binning pixels */
	u32 dig_y;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

#define IMX06A_MODE_REGS(x)	.regs = x, .num_regs = ARRAY_SIZE(x)

/*
 * DCG-HDR reads every line at both conversion gains, so its line period must be
 * longer than the single-read formula gives for its LLPCK: below 182 INCK cycles
 * (7.58 us) the merged output is all zeros, at any lane rate and either ratio
 * (measured). 26784 = 184 INCK at the 2184 MHz IVTCK, ~1% margin.
 */
#define IMX06A_DCG_RO_LLPCK		26784U

/* Per mode family: line length, CIT and gain rules, register table. */
#define IMX06A_FULL10	.code = MEDIA_BUS_FMT_SRGGB10_1X10, .llpck = 20128,	\
			.fll_step = 2, .cit_rule = IMX06A_CIT_FULL,		\
			.exp_min = 6, .exp_step = 2,				\
			.gain_max = 15360, .gain_step = 4, .bin = 1,		\
			IMX06A_MODE_REGS(imx06a_mode_full10_regs)
#define IMX06A_FULL12	.code = MEDIA_BUS_FMT_SRGGB12_1X12, .llpck = 22880,	\
			.fll_step = 2, .cit_rule = IMX06A_CIT_FULL,		\
			.exp_min = 6, .exp_step = 2,				\
			.gain_max = 15360, .gain_step = 4, .bin = 1,		\
			IMX06A_MODE_REGS(imx06a_mode_full12_regs)
#define IMX06A_BIN10	.code = MEDIA_BUS_FMT_SRGGB10_1X10, .llpck = 15872,	\
			.fll_step = 4, .cit_rule = IMX06A_CIT_BIN,		\
			.exp_min = 4, .exp_step = 4,				\
			.gain_max = 16128, .gain_step = 4, .bin = 2,		\
			IMX06A_MODE_REGS(imx06a_mode_bin_regs)
#define IMX06A_BIN4	.code = MEDIA_BUS_FMT_SRGGB10_1X10, .llpck = 4416,	\
			.fll_step = 8, .cit_rule = IMX06A_CIT_BIN,		\
			.exp_min = 24, .exp_step = 8,				\
			.gain_max = 16128, .gain_step = 4, .bin = 4,		\
			IMX06A_MODE_REGS(imx06a_mode_bin4_regs)
#define IMX06A_DCG4	.code = MEDIA_BUS_FMT_SRGGB12_1X12, .llpck = 19232,	\
			.ro_llpck = IMX06A_DCG_RO_LLPCK,			\
			.fll_step = 4, .cit_rule = IMX06A_CIT_BIN,		\
			.exp_min = 4, .exp_step = 4,				\
			.gain_max = 15360, .gain_step = 16, .hdr_ratio = 4,	\
			.hdr = IMX06A_HDR_DCG,					\
			.bin = 2, IMX06A_MODE_REGS(imx06a_mode_dcg4_regs)
#define IMX06A_DCG16	.code = MEDIA_BUS_FMT_SRGGB14_1X14, .llpck = 19232,	\
			.ro_llpck = IMX06A_DCG_RO_LLPCK,			\
			.fll_step = 4, .cit_rule = IMX06A_CIT_BIN,		\
			.exp_min = 4, .exp_step = 4,				\
			.gain_max = 12288, .gain_step = 64, .hdr_ratio = 16,	\
			.hdr = IMX06A_HDR_DCG,					\
			.bin = 2, IMX06A_MODE_REGS(imx06a_mode_dcg16_regs)
#define IMX06A_QHDR	.code = MEDIA_BUS_FMT_SRGGB12_1X12, .llpck = 17664,	\
			.fll_step = 2, .cit_rule = IMX06A_CIT_QBC,		\
			.exp_min = 24, .exp_step = 8,				\
			.gain_max = 15872, .gain_step = 4,			\
			.hdr = IMX06A_HDR_QBC,					\
			.bin = 2, IMX06A_MODE_REGS(imx06a_mode_qhdr_regs)

/*
 * Minimum frame lengths are those of the 4:3 readouts; the crops keep the
 * same blanking. 4x4: complete frames from 856 lines at 720p and 1192 at
 * 1080p (measured; shorter frames lose their last rows), +8 for margin; frame
 * length and exposure act in steps of 8 lines there (measured), and the
 * minimum exposure stays clear of the prohibited small binning values. The
 * vertical window is free in 32-row steps; the horizontal window must stay
 * 0..8191, so the crops narrow the field with the digital crop.
 * The first entry of each HDR family is its default.
 */
static const struct imx06a_mode imx06a_modes[] = {
	/* HDR off */
	{ IMX06A_FULL10, .name = "full 4:3 RAW10", .width = 8192, .height = 6144,
	  .fll_min = 6370, .y_start = 0, .rows = 6144 },
	{ IMX06A_FULL10, .name = "full 16:9 RAW10", .width = 8192, .height = 4608,
	  .fll_min = 4834, .y_start = 768, .rows = 4608 },
	{ IMX06A_FULL12, .name = "full 4:3 RAW12", .width = 8192, .height = 6144,
	  .fll_min = 6370, .y_start = 0, .rows = 6144 },
	{ IMX06A_FULL12, .name = "full 16:9 RAW12", .width = 8192, .height = 4608,
	  .fll_min = 4834, .y_start = 768, .rows = 4608 },
	{ IMX06A_BIN10, .name = "binned 4:3 RAW10", .width = 4096, .height = 3072,
	  .fll_min = 3220, .y_start = 0, .rows = 6144 },
	{ IMX06A_BIN10, .name = "binned 16:9 RAW10", .width = 4096, .height = 2304,
	  .fll_min = 2452, .y_start = 768, .rows = 4608 },
	{ IMX06A_BIN10, .name = "binned 4K UHD crop RAW10", .width = 3840,
	  .height = 2160, .fll_min = 2324, .y_start = 896, .rows = 4352,
	  .dig_x = 128, .dig_y = 8 },
	{ IMX06A_BIN4, .name = "4x4 binned 1080p crop RAW10", .width = 1920,
	  .height = 1080, .fll_min = 1200, .y_start = 896, .rows = 4352,
	  .dig_x = 64, .dig_y = 4 },
	{ IMX06A_BIN4, .name = "4x4 binned 720p crop RAW10", .width = 1280,
	  .height = 720, .fll_min = 864, .y_start = 1536, .rows = 3072,
	  .dig_x = 384, .dig_y = 24 },
	/* DCG-HDR */
	{ IMX06A_DCG4, .name = "DCG-HDR 4:1 4:3 RAW12", .width = 4096,
	  .height = 3072, .fll_min = 3272, .y_start = 0, .rows = 6144 },
	{ IMX06A_DCG4, .name = "DCG-HDR 4:1 16:9 RAW12", .width = 4096,
	  .height = 2304, .fll_min = 2504, .y_start = 768, .rows = 4608 },
	{ IMX06A_DCG16, .name = "DCG-HDR 16:1 4:3 RAW14", .width = 4096,
	  .height = 3072, .fll_min = 3272, .y_start = 0, .rows = 6144 },
	{ IMX06A_DCG16, .name = "DCG-HDR 16:1 16:9 RAW14", .width = 4096,
	  .height = 2304, .fll_min = 2504, .y_start = 768, .rows = 4608 },
	/* QBC-HDR */
	{ IMX06A_QHDR, .name = "QBC-HDR 4:3 RAW12", .width = 4096, .height = 3072,
	  .fll_min = 3240, .y_start = 0, .rows = 6144 },
	{ IMX06A_QHDR, .name = "QBC-HDR 16:9 RAW12", .width = 4096, .height = 2304,
	  .fll_min = 2472, .y_start = 768, .rows = 4608 },
};

/*
 * Named link presets ("link-frequency=DCI60P" overlay parameter, or written to
 * link_mbps): a (mode, frame rate) target that must be hit exactly. See
 * imx06a_preset_lane_hz().
 */
static const struct imx06a_link_preset {
	const char *name;
	u32 width, height, fps;
} imx06a_link_presets[] = {
	{ "DCI60P", 4096, 2304, 60 },
	{ "DCI50P", 4096, 2304, 50 },
	{ "DCI30P", 4096, 2304, 30 },
	{ "UHD60P", 3840, 2160, 60 },
	{ "UHD50P", 3840, 2160, 50 },
	{ "UHD30P", 3840, 2160, 30 },
};

/*
 * Sync output signals. Exposure strobe: high from the first row's exposure
 * start to the end of readout (exposure + readout lines), once per frame.
 * V / H sync: short active-low pulses per frame / line.
 */
enum imx06a_strobe {
	IMX06A_STROBE_OFF,
	IMX06A_STROBE_EXPOSURE,
	IMX06A_STROBE_VSYNC,
	IMX06A_STROBE_HSYNC,
};

static const char * const imx06a_strobe_menu[] = {
	[IMX06A_STROBE_OFF] = "Off",
	[IMX06A_STROBE_EXPOSURE] = "Exposure strobe",
	[IMX06A_STROBE_VSYNC] = "V sync pulse",
	[IMX06A_STROBE_HSYNC] = "H sync pulse",
};

static const char * const imx06a_hdr_menu[] = {
	[IMX06A_HDR_OFF] = "Off",
	[IMX06A_HDR_DCG] = "DCG-HDR (dual conversion gain, single exposure)",
	[IMX06A_HDR_QBC] = "QBC-HDR (quad long/short, ratio 4)",
};

static const char * const imx06a_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"100% Colour Bars",
	"Fade to Grey Colour Bars",
	"PN9",
};

static const char * const imx06a_supply_name[] = {
	"vana",		/* 2.9 V / 1.8 V analogue */
	"vdig",		/* 0.83 V digital core */
	"vif",		/* 1.8 V interface */
};

#define IMX06A_NUM_SUPPLIES ARRAY_SIZE(imx06a_supply_name)

enum { IMX06A_PAD_IMAGE, IMX06A_PAD_PD, IMX06A_NUM_PADS };

struct imx06a_clocks {
	u32 ivt_prediv;
	u32 ivt_mpy;
	u32 iop_prediv;
	u32 iop_mpy;
	u32 iop_div;
	u64 ivtck;		/* Hz */
	u32 llpck;		/* LINE_LENGTH_PCK (the mode's shortest legal value) */
	u32 llinck_min;		/* shortest line in INCK cycles (readout / link) */
	u64 lane_hz;		/* actual lane rate (exact PLL output) */
};

struct imx06a {
	struct v4l2_subdev sd;
	struct media_pad pads[IMX06A_NUM_PADS];
	struct device *dev;
	struct regmap *regmap;

	struct clk *xclk;
	u32 inck;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX06A_NUM_SUPPLIES];

	u64 link_req_hz;	/* requested lane rate */
	struct imx06a_clocks clk;	/* synthesised for the current mode */
	s64 link_freq_menu[1];	/* LINK_FREQ = actual lane rate / 2 (DDR) */

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *again;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *wb_gains;
	struct v4l2_ctrl *hdr_ctrl;
	struct v4l2_ctrl *wdr_ctrl;
	struct v4l2_ctrl *gpo_ctrl;
	struct v4l2_ctrl *testout_ctrl;

	const struct imx06a_mode *mode;
	enum imx06a_hdr hdr;	/* the advertised mode family */
	bool pdaf;		/* PD pad exposed */
	u32 gpo_def;		/* sync output control defaults (DT) */
	u32 testout_def;

	/* Global registers are written once after each power-on. */
	bool globals_written;
};

static inline struct imx06a *to_imx06a(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx06a, sd);
}

/* --------------------------------------------------------------------------
 * Module parameters
 * --------------------------------------------------------------------------
 */
static bool pdaf;
module_param(pdaf, bool, 0444);
MODULE_PARM_DESC(pdaf, "Expose the phase-detect output on a second pad (all devices)");

/* Bring-up: report a different packet type on the PD pad (0x12 = embedded data). */
static uint pd_dt = IMX06A_PD_DT;
module_param(pd_dt, uint, 0644);
MODULE_PARM_DESC(pd_dt, "Bring-up: CSI-2 data type of the PD pad (default 0x30)");
static uint pd_vc;
module_param(pd_vc, uint, 0644);
MODULE_PARM_DESC(pd_vc, "Bring-up: CSI-2 virtual channel of the PD pad (default 0)");

/*
 * Bring-up: "addr=val,..." 8-bit writes applied just before streaming starts,
 * after the mode tables and controls. Load-time only (reload the module): a
 * writable charp parameter is freed by sysfs writes while it may be parsed.
 */
static char *dbg_regs;
module_param(dbg_regs, charp, 0444);
MODULE_PARM_DESC(dbg_regs,
		 "Bring-up: register writes before stream-on (addr=val,...), load-time only");

static bool probe_stay_on;
module_param(probe_stay_on, bool, 0444);
MODULE_PARM_DESC(probe_stay_on,
		 "Bring-up: keep probing and stay powered even if the chip id read fails");

/*
 * Lane rate. The request comes from, in priority order: link_mbps given at
 * module load, the overlay's link-frequency (Hz or preset name), the endpoint's
 * link-frequencies[0]. link_mbps reads back the actual rate and can be written
 * between streams; its setter has no device context, so it acts on the first
 * bound device (imx06a_self).
 */
static unsigned int link_mbps;
static u64 link_param_hz;	/* rate given at module load, 0 = none */
static struct imx06a *imx06a_self;
static DEFINE_MUTEX(imx06a_self_lock);

/* --------------------------------------------------------------------------
 * Mode and format helpers
 * --------------------------------------------------------------------------
 */
static bool imx06a_code_bpp(u32 code, unsigned int *bpp)
{
	switch (code) {
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		*bpp = 10;
		return true;
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		*bpp = 12;
		return true;
	case MEDIA_BUS_FMT_SRGGB14_1X14:
	case MEDIA_BUS_FMT_SGRBG14_1X14:
	case MEDIA_BUS_FMT_SGBRG14_1X14:
	case MEDIA_BUS_FMT_SBGGR14_1X14:
		*bpp = 14;
		return true;
	default:
		return false;
	}
}

static u32 imx06a_base_code(u32 code)
{
	unsigned int bpp = 10;

	imx06a_code_bpp(code, &bpp);
	if (bpp == 14)
		return MEDIA_BUS_FMT_SRGGB14_1X14;
	return bpp == 12 ? MEDIA_BUS_FMT_SRGGB12_1X12 : MEDIA_BUS_FMT_SRGGB10_1X10;
}

static unsigned int imx06a_mode_bpp(const struct imx06a_mode *m)
{
	unsigned int bpp = 10;

	imx06a_code_bpp(m->code, &bpp);
	return bpp;
}

static u32 imx06a_flip_code(u32 base, bool hflip, bool vflip)
{
	static const u32 c10[2][2] = {
		{ MEDIA_BUS_FMT_SRGGB10_1X10, MEDIA_BUS_FMT_SGRBG10_1X10 },
		{ MEDIA_BUS_FMT_SGBRG10_1X10, MEDIA_BUS_FMT_SBGGR10_1X10 },
	};
	static const u32 c12[2][2] = {
		{ MEDIA_BUS_FMT_SRGGB12_1X12, MEDIA_BUS_FMT_SGRBG12_1X12 },
		{ MEDIA_BUS_FMT_SGBRG12_1X12, MEDIA_BUS_FMT_SBGGR12_1X12 },
	};
	static const u32 c14[2][2] = {
		{ MEDIA_BUS_FMT_SRGGB14_1X14, MEDIA_BUS_FMT_SGRBG14_1X14 },
		{ MEDIA_BUS_FMT_SGBRG14_1X14, MEDIA_BUS_FMT_SBGGR14_1X14 },
	};

	if (base == MEDIA_BUS_FMT_SRGGB14_1X14)
		return c14[vflip][hflip];
	if (base == MEDIA_BUS_FMT_SRGGB12_1X12)
		return c12[vflip][hflip];
	return c10[vflip][hflip];
}

static u32 imx06a_current_code(const struct imx06a *imx06a, u32 base)
{
	return imx06a_flip_code(base, imx06a->hflip->val, imx06a->vflip->val);
}

/*
 * Nearest mode of an HDR family to a requested code and size. A code the family
 * does not offer falls back to the nearest size of any code it has, so set_fmt
 * always lands on a mode.
 */
static const struct imx06a_mode *imx06a_find_mode(enum imx06a_hdr hdr, u32 base,
						  u32 w, u32 h)
{
	const struct imx06a_mode *best = NULL, *any = NULL;
	u64 best_d = U64_MAX, any_d = U64_MAX;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(imx06a_modes); i++) {
		const struct imx06a_mode *m = &imx06a_modes[i];
		u64 d;

		if (m->hdr != hdr)
			continue;
		d = (u64)abs_diff(m->width, w) + abs_diff(m->height, h);
		if (d < any_d) {
			any_d = d;
			any = m;
		}
		if (m->code == base && d < best_d) {
			best_d = d;
			best = m;
		}
	}
	return best ?: any;
}

static const struct imx06a_mode *imx06a_default_mode(enum imx06a_hdr hdr)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(imx06a_modes); i++)
		if (imx06a_modes[i].hdr == hdr)
			return &imx06a_modes[i];
	return &imx06a_modes[0];
}

/* Native-pixel rectangle a mode covers (V4L2_SEL_TGT_CROP). */
static struct v4l2_rect imx06a_mode_crop(const struct imx06a_mode *m)
{
	return (struct v4l2_rect) {
		.left = m->dig_x * m->bin,
		.top = m->y_start + m->dig_y * m->bin,
		.width = m->width * m->bin,
		.height = m->height * m->bin,
	};
}

static void imx06a_fill_fmt(const struct imx06a *imx06a,
			    const struct imx06a_mode *mode,
			    struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = imx06a_current_code(imx06a, mode->code);
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static void imx06a_fill_pd_fmt(struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->width = IMX06A_PD_PAD_WIDTH;
	fmt->height = 1;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

/* Point a pad state at a mode: image format and crop, and the PD pad if any. */
static void imx06a_state_set_mode(const struct imx06a *imx06a,
				  struct v4l2_subdev_state *state,
				  const struct imx06a_mode *mode)
{
	imx06a_fill_fmt(imx06a, mode,
			v4l2_subdev_state_get_format(state, IMX06A_PAD_IMAGE));
	*v4l2_subdev_state_get_crop(state, IMX06A_PAD_IMAGE) = imx06a_mode_crop(mode);
	if (imx06a->pdaf)
		imx06a_fill_pd_fmt(v4l2_subdev_state_get_format(state, IMX06A_PAD_PD));
}

/*
 * PD stream geometry per mode (RAW10 on VC0 / DT 0x30):
 *  - all-pixel PD (every mode but full-res AD12; PHASE_PIX = 1): one packet per
 *    4 image lines of interleaved L/R values, one per binned-equivalent output
 *    column (4096 at full width, following the horizontal crop);
 *  - sparse PD (full-res AD12 only; PDAF_TYPE2 = 1): two packets of 1016 values
 *    per 4 image lines, one sample per 8 columns and 2 rows, covering array rows
 *    64..6079 clipped to the readout window; L and R alternate in row pairs.
 */
struct imx06a_pd_geom {
	u32 px;		/* values per packet */
	u32 rows;	/* packets per frame */
	u32 pkts;	/* packets per IMX06A_PD_LINE_GROUP image lines */
	bool phase_pix;	/* all-pixel PD; else sparse */
};

static void imx06a_pd_geom(const struct imx06a_mode *mode, struct imx06a_pd_geom *g)
{
	bool full = mode->cit_rule == IMX06A_CIT_FULL;

	if (full && mode->code == MEDIA_BUS_FMT_SRGGB12_1X12) {
		g->px = IMX06A_PD_SPARSE_PX;
		g->rows = (min(mode->y_start + mode->rows, IMX06A_PD_SPARSE_Y1) -
			   max(mode->y_start, IMX06A_PD_SPARSE_Y0)) / 2;
		g->pkts = 2;
		g->phase_pix = false;
		return;
	}
	g->px = min(full ? mode->width / 2 : mode->width, IMX06A_PD_PIXELS);
	g->rows = DIV_ROUND_UP(mode->height, IMX06A_PD_LINE_GROUP);
	g->pkts = 1;
	g->phase_pix = true;
}

/* --------------------------------------------------------------------------
 * Clock synthesis
 * --------------------------------------------------------------------------
 */

/* The PLL reference INCK / prediv must stay within 12..24 MHz. */
static unsigned int imx06a_pll_prediv(u32 inck)
{
	return inck > 24 * HZ_PER_MHZ ? 2 : 1;
}

/*
 * Lane rate: the IOP PLL output closest to the request (or the lowest one at
 * or above it), keeping the PLL reference at the full INCK; a lower reference
 * would reach finer rates with more jitter. With a 24 MHz
 * INCK this is any multiple of 12 Mbps from 1080 to 2244 Mbps, plus coarser
 * grids below.
 */
static int imx06a_round_lane_rate(u32 inck, u64 req, bool at_least,
				  struct imx06a_clocks *c)
{
	static const u32 iop_divs[] = { 2, 1, 4, 8 };
	unsigned int i, pre = imx06a_pll_prediv(inck);
	u64 f = inck / pre, best_err = U64_MAX;

	for (i = 0; i < ARRAY_SIZE(iop_divs); i++) {
		u32 div = iop_divs[i];
		u64 mpy = at_least ? DIV64_U64_ROUND_UP(req * div, f) :
				     DIV64_U64_ROUND_CLOSEST(req * div, f);
		u64 vco, lane, err;

		mpy = clamp_t(u64, mpy, IMX06A_IOP_MPY_MIN, IMX06A_IOP_MPY_MAX);
		vco = f * mpy;
		if (vco < IMX06A_IOPCK_MIN_HZ || vco > IMX06A_IOPCK_MAX_HZ ||
		    vco % div)
			continue;
		lane = vco / div;
		if (at_least && lane < req)
			continue;
		err = abs_diff(lane, req);
		if (err < best_err) {
			best_err = err;
			c->iop_prediv = pre;
			c->iop_mpy = mpy;
			c->iop_div = div;
			c->lane_hz = lane;
		}
	}
	return best_err == U64_MAX ? -EINVAL : 0;
}

/* Readout floor (SRM 4.5.1): ceil(LLPCK * INCK / (1.6 * IVTCK)) INCK cycles. */
static u32 imx06a_readout_inck(u32 inck, u64 ivtck, const struct imx06a_mode *mode)
{
	u32 llpck = max(mode->llpck, mode->ro_llpck);

	return DIV64_U64_ROUND_UP((u64)llpck * inck * 5, ivtck * 8);
}

/* INCK cycles for pkts CSI-2 packets of payload_ui each, per group lines. */
static u32 imx06a_packet_inck(u32 inck, u64 lane_hz, u32 payload_ui, u32 pkts,
			      u32 group)
{
	return DIV64_U64_ROUND_UP((u64)pkts * (payload_ui + IMX06A_LINE_OH_UI) *
				  inck, lane_hz * group) +
	       DIV_ROUND_UP(pkts * IMX06A_LINE_OH_NS * (inck / 1000),
			    1000000 * group);
}

static int imx06a_synth_clocks(u32 inck, u64 req_hz,
			       const struct imx06a_mode *mode, bool pd_on,
			       struct imx06a_clocks *c)
{
	u32 link;
	u64 fin;
	int ret;

	if (req_hz < IMX06A_LINK_MBPS_MIN * HZ_PER_MHZ ||
	    req_hz > IMX06A_LINK_MBPS_MAX * HZ_PER_MHZ ||
	    inck < IMX06A_INCK_MIN_HZ || inck > IMX06A_INCK_MAX_HZ)
		return -ERANGE;

	memset(c, 0, sizeof(*c));
	ret = imx06a_round_lane_rate(inck, req_hz, false, c);
	if (ret)
		return ret;

	/* IVT: the fastest IVTCK the PLL allows. */
	c->ivt_prediv = imx06a_pll_prediv(inck);
	fin = inck / c->ivt_prediv;
	c->ivt_mpy = min_t(u64, IMX06A_IVT_MPY_MAX,
			   div64_u64(IMX06A_IVTCK_MAX_HZ, fin));
	c->ivtck = fin * c->ivt_mpy;
	if (c->ivt_mpy < IMX06A_IVT_MPY_MIN || c->ivtck < IMX06A_IVTCK_MIN_HZ)
		return -ERANGE;
	c->llpck = mode->llpck;

	/* Link floor: one output line, plus the PD packets per line group. */
	link = imx06a_packet_inck(inck, c->lane_hz,
				  mode->width * imx06a_mode_bpp(mode) / 4, 1, 1);
	if (pd_on) {
		struct imx06a_pd_geom g;

		imx06a_pd_geom(mode, &g);
		link += imx06a_packet_inck(inck, c->lane_hz,
					   g.px * IMX06A_PD_BPP / 4, g.pkts,
					   IMX06A_PD_LINE_GROUP);
	}
	/* HBLANK (line minus width) must also stay positive. */
	c->llinck_min = max3(imx06a_readout_inck(inck, c->ivtck, mode), link,
			     mode->width / IMX06A_PIX_PER_INCK + 1);
	return 0;
}

/*
 * Lowest exact PLL lane rate (up to IMX06A_LINK_MBPS_MAX) whose link floor,
 * including the PD packets when pd_on, fits a line of h INCK cycles; 0 if none.
 * Walks the PLL grid upwards: at most a few hundred steps.
 */
static u64 imx06a_lowest_rate_for_line(u32 inck, const struct imx06a_mode *m,
				       bool pd_on, u32 h)
{
	u64 req = (u64)IMX06A_LINK_MBPS_MIN * HZ_PER_MHZ;
	struct imx06a_clocks r, c;

	while (!imx06a_round_lane_rate(inck, req, true, &r) &&
	       r.lane_hz <= (u64)IMX06A_LINK_MBPS_MAX * HZ_PER_MHZ) {
		if (!imx06a_synth_clocks(inck, r.lane_hz, m, pd_on, &c) &&
		    c.llinck_min <= h)
			return c.lane_hz;
		req = r.lane_hz + 1;
	}
	return 0;
}

/*
 * Lane rate for a named preset: the lowest exact PLL rate at which the preset's
 * mode reaches its frame rate EXACTLY, with an integer line length h (INCK
 * cycles) times a legal frame length equal to the frame period. libcamera's
 * imx06a helper picks such an (h, frame length) pair when the rate is pinned.
 * "MAX" is the fastest rate verified error-free on the RP1.
 */
static int imx06a_preset_lane_hz(struct device *dev, u32 inck, bool pd_on,
				 const char *name, u64 *lane_hz)
{
	const struct imx06a_link_preset *p = NULL;
	const struct imx06a_mode *m = NULL;
	struct imx06a_clocks c;
	u32 llinck_max, ro, h_exact = 0;
	unsigned int i;
	u64 rate = 0;
	int ret;

	if (!strcasecmp(name, "MAX")) {
		*lane_hz = (u64)IMX06A_LINK_MBPS_MAX_CLEAN * HZ_PER_MHZ;
		return 0;
	}
	for (i = 0; i < ARRAY_SIZE(imx06a_link_presets); i++)
		if (!strcasecmp(name, imx06a_link_presets[i].name))
			p = &imx06a_link_presets[i];
	for (i = 0; p && i < ARRAY_SIZE(imx06a_modes); i++)
		if (imx06a_modes[i].hdr == IMX06A_HDR_OFF &&
		    imx06a_modes[i].width == p->width &&
		    imx06a_modes[i].height == p->height)
			m = &imx06a_modes[i];
	if (!m) {
		dev_err(dev, "unknown link preset \"%s\" (MAX, DCI60P/50P/30P, UHD60P/50P/30P or Hz)\n",
			name);
		return -EINVAL;
	}

	/* Longest line that still gives the frame rate at the minimum FLL. */
	llinck_max = div_u64(inck, p->fps * m->fll_min);
	/* The readout floor does not depend on the lane rate. */
	ret = imx06a_synth_clocks(inck, (u64)IMX06A_LINK_MBPS_MAX_CLEAN * HZ_PER_MHZ,
				  m, pd_on, &c);
	if (ret)
		return ret;
	ro = imx06a_readout_inck(inck, c.ivtck, m);

	/*
	 * Exact line: the longest h dividing the period T = INCK / fps with a
	 * frame length T / h that is a multiple of 4 (the coarsest VBLANK step)
	 * and at least the mode minimum. Without one, or if no lane rate reaches
	 * it, fall back to the longest line that just reaches the frame rate.
	 */
	if (!(inck % p->fps)) {
		u32 t = inck / p->fps, h;

		for (h = llinck_max;
		     h >= ro && h > m->width / IMX06A_PIX_PER_INCK; h--)
			if (!(t % h) && !((t / h) % 4) && t / h >= m->fll_min) {
				h_exact = h;
				break;
			}
	}
	if (h_exact)
		rate = imx06a_lowest_rate_for_line(inck, m, pd_on, h_exact);
	if (!rate) {
		rate = imx06a_lowest_rate_for_line(inck, m, pd_on, llinck_max);
		if (rate && h_exact)
			dev_warn(dev, "link preset %s: exact %u fps unreachable%s\n",
				 name, p->fps, pd_on ? " with PD on" : "");
	}
	if (!rate) {
		dev_err(dev, "link preset %s: %ux%u cannot reach %u fps%s\n", name,
			p->width, p->height, p->fps, pd_on ? " with PD on" : "");
		return -ERANGE;
	}
	if (rate > (u64)IMX06A_LINK_MBPS_MAX_CLEAN * HZ_PER_MHZ)
		dev_warn(dev, "link preset %s needs %llu kbps, above the %u Mbps verified on the RP1\n",
			 name, div_u64(rate, 1000), IMX06A_LINK_MBPS_MAX_CLEAN);
	*lane_hz = rate;
	return 0;
}

/* --------------------------------------------------------------------------
 * Controls
 * --------------------------------------------------------------------------
 */

/* Line period in INCK cycles for the current HBLANK (one step = one INCK). */
static u32 imx06a_llinck(const struct imx06a *imx06a)
{
	return (imx06a->mode->width + imx06a->hblank->val) / IMX06A_PIX_PER_INCK;
}

/* Frame length in lines as exposed to userspace (may exceed 16 bits). */
static u32 imx06a_fll(const struct imx06a *imx06a)
{
	return imx06a->mode->height + imx06a->vblank->val;
}

/* Round an exposure down to the mode's grid (steps counted from its minimum). */
static u32 imx06a_round_exp(const struct imx06a_mode *mode, u32 lines)
{
	return mode->exp_min + rounddown(max(lines, mode->exp_min) - mode->exp_min,
					 mode->exp_step);
}

/*
 * Table 5-25: small COARSE_INTEG_TIME values are prohibited. Bump one to the
 * next legal value; this only affects exposures of a few lines.
 */
static u32 imx06a_legal_cit(const struct imx06a_mode *mode, u32 cit)
{
	switch (mode->cit_rule) {
	case IMX06A_CIT_FULL:
		if (cit >= 8 && cit <= 28 && !(cit % 4))
			cit += 2;
		break;
	case IMX06A_CIT_BIN:
		if (cit == 8 || cit == 16)
			cit += 4;
		break;
	case IMX06A_CIT_QBC:
		/* short = long / 4 may not be 8 (long 24 -> short 6 is legal) */
		if (cit == 32)
			cit += 8;
		break;
	}
	return cit;
}

/* V4L2 (LSG-referenced) gain code -> ANA_GAIN_GLOBAL register value. */
static u32 imx06a_hw_gain(const struct imx06a_mode *mode, u32 code)
{
	if (!mode->hdr_ratio)
		return code;
	return IMX06A_ANA_GAIN_SCALE - IMX06A_ANA_GAIN_SCALE / mode->hdr_ratio +
	       code / mode->hdr_ratio;
}

/*
 * Frame length, exposure and long-exposure shift, written together under group
 * hold so they always describe the same frame.
 */
static int imx06a_write_timing(struct imx06a *imx06a)
{
	const struct imx06a_mode *mode = imx06a->mode;
	u32 fll = imx06a_fll(imx06a);
	u32 fll_max = rounddown(IMX06A_FRM_LENGTH_MAX, mode->fll_step);
	u32 fll_reg, cit, shift = 0;
	int ret = 0;

	while ((fll >> shift) > fll_max && shift < IMX06A_LONG_EXP_SHIFT_MAX)
		shift++;
	fll_reg = min(roundup(DIV_ROUND_UP(fll, 1U << shift), mode->fll_step),
		      fll_max);

	cit = min_t(u32, imx06a->exposure->val >> shift,
		    fll_reg - IMX06A_EXPOSURE_MARGIN);
	cit = imx06a_legal_cit(mode, imx06a_round_exp(mode, cit));

	cci_write(imx06a->regmap, IMX06A_REG_GROUP_HOLD, 1, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_CIT_LSHIFT, shift, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_FRM_LENGTH, fll_reg, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_EXPOSURE, cit, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_GROUP_HOLD, 0, &ret);
	return ret;
}

/*
 * Black level in 12-bit LSBs. Normal modes: the pedestal register alone. HDR:
 * the target T in output LSBs comes from pedestal / 4 + offset / 2^(14 - bpp);
 * use the offset first (keeping the pedestal at its default 64 output LSBs)
 * and the pedestal for the rest, so the default reproduces the stock HDR setting
 * (offset 768 at 12 bit, 960 at 14 bit) and black is 4096 in 16-bit units.
 */
static int imx06a_write_black_level(struct imx06a *imx06a, u32 level)
{
	const struct imx06a_mode *mode = imx06a->mode;
	u32 ped = level, off = 0;
	int ret = 0;

	if (mode->hdr != IMX06A_HDR_OFF) {
		unsigned int bpp = imx06a_mode_bpp(mode), s = 14 - bpp;
		u32 t = level << (bpp - 12);		/* output LSBs */

		off = t > 64 ? min((t - 64) << s, IMX06A_PEDESTAL_OFFSET_MAX) : 0;
		ped = min(4 * t - ((4 * off) >> s), IMX06A_DATA_PEDESTAL_MAX);
	}

	cci_write(imx06a->regmap, IMX06A_REG_GROUP_HOLD, 1, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_DATA_PEDESTAL, ped, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_PEDESTAL_OFFSET_EN,
		  mode->hdr != IMX06A_HDR_OFF, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_PEDESTAL_OFFSET, off, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_GROUP_HOLD, 0, &ret);
	return ret;
}

static int imx06a_write_wb_gains(struct imx06a *imx06a)
{
	/* V4L2_CID_NOTIFY_GAINS order: B, Gb, Gr, R */
	const u32 *g = imx06a->wb_gains->p_new.p_u32;
	int ret = 0;

	cci_write(imx06a->regmap, IMX06A_REG_GROUP_HOLD, 1, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_ABS_GAIN_B, g[0], &ret);
	cci_write(imx06a->regmap, IMX06A_REG_ABS_GAIN_GB, g[1], &ret);
	cci_write(imx06a->regmap, IMX06A_REG_ABS_GAIN_GR, g[2], &ret);
	cci_write(imx06a->regmap, IMX06A_REG_ABS_GAIN_R, g[3], &ret);
	cci_write(imx06a->regmap, IMX06A_REG_GROUP_HOLD, 0, &ret);
	return ret;
}

static int imx06a_write_sync_outputs(struct imx06a *imx06a)
{
	static const u8 mon[] = {
		[IMX06A_STROBE_OFF] = 0,
		[IMX06A_STROBE_EXPOSURE] = IMX06A_MON_LONG_FSTROBE,
		[IMX06A_STROBE_VSYNC] = IMX06A_MON_MNTTEST2,
		[IMX06A_STROBE_HSYNC] = IMX06A_MON_MNTTEST1,
	};
	u32 gpo = imx06a->gpo_ctrl->val, tout = imx06a->testout_ctrl->val;
	bool strobe = gpo == IMX06A_STROBE_EXPOSURE || tout == IMX06A_STROBE_EXPOSURE;
	bool vsync = gpo == IMX06A_STROBE_VSYNC || tout == IMX06A_STROBE_VSYNC;
	bool hsync = gpo == IMX06A_STROBE_HSYNC || tout == IMX06A_STROBE_HSYNC;
	int ret = 0;

	cci_write(imx06a->regmap, IMX06A_REG_FSTROBE_MODE,
		  strobe ? IMX06A_FSTROBE_LONG : 0, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_MON_SEL,
		  gpo || tout ? 0 : IMX06A_MON_SEL_DEFAULT, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_GPIO_SEL, 0, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_MNTTEST1_SEL,
		  hsync ? IMX06A_MNTTEST_HSYNC : 0, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_MNTTEST2_SEL,
		  vsync ? IMX06A_MNTTEST_VSYNC : 0, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_MON_GPO, mon[gpo], &ret);
	cci_write(imx06a->regmap, IMX06A_REG_MON_TESTOUT, mon[tout], &ret);
	cci_write(imx06a->regmap, IMX06A_REG_IO_CTRL_GPO, 0, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_IO_CTRL_TESTOUT, 0, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_GPO_CTRL_SEL, 0, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_TESTOUT_CTRL_SEL, 0, &ret);
	return ret;
}

static int imx06a_update_exposure_range(struct imx06a *imx06a)
{
	const struct imx06a_mode *mode = imx06a->mode;
	u32 max = imx06a_round_exp(mode, imx06a_fll(imx06a) - IMX06A_EXPOSURE_MARGIN);
	u32 def = imx06a_round_exp(mode, min_t(u32, imx06a->exposure->val, max));

	return __v4l2_ctrl_modify_range(imx06a->exposure, mode->exp_min, max,
					mode->exp_step, def);
}

/*
 * Re-synthesise the clocks for the current mode at the requested lane rate,
 * publish the actual rate (LINK_FREQ, link_mbps) and re-range HBLANK: minimum
 * = the readout/link floor, step = one INCK cycle. HBLANK is driven back to the
 * minimum so a longer line from a previous mode or rate does not linger.
 */
static int imx06a_update_clocks(struct imx06a *imx06a)
{
	const struct imx06a_mode *mode = imx06a->mode;
	struct imx06a_clocks clk;
	u32 hmin, hmax;
	int ret;

	ret = imx06a_synth_clocks(imx06a->inck, imx06a->link_req_hz, mode,
				  imx06a->pdaf, &clk);
	if (ret)
		return ret;
	imx06a->clk = clk;
	imx06a->link_freq_menu[0] = clk.lane_hz / 2;
	link_mbps = DIV64_U64_ROUND_CLOSEST(clk.lane_hz, HZ_PER_MHZ);

	hmin = clk.llinck_min * IMX06A_PIX_PER_INCK - mode->width;
	hmax = IMX06A_LLINCK_MAX * IMX06A_PIX_PER_INCK - mode->width;
	ret = __v4l2_ctrl_modify_range(imx06a->hblank, hmin, hmax,
				       IMX06A_PIX_PER_INCK, hmin);
	return ret ?: __v4l2_ctrl_s_ctrl(imx06a->hblank, hmin);
}

/* Re-derive every mode-dependent control range. Caller holds the state lock. */
static int imx06a_set_framing_limits(struct imx06a *imx06a)
{
	const struct imx06a_mode *mode = imx06a->mode;
	u32 vblank_min = mode->fll_min - mode->height;
	u32 vblank_max = (rounddown(IMX06A_FRM_LENGTH_MAX, mode->fll_step)
			  << IMX06A_LONG_EXP_SHIFT_MAX) - mode->height;
	int ret;

	ret = imx06a_update_clocks(imx06a);
	if (!ret)
		ret = __v4l2_ctrl_modify_range(imx06a->vblank, vblank_min,
					       vblank_max, mode->fll_step,
					       vblank_min);
	if (!ret)
		ret = __v4l2_ctrl_s_ctrl(imx06a->vblank, vblank_min);
	/* s_ctrl(VBLANK) only re-ranges the exposure when the value changes. */
	if (!ret)
		ret = imx06a_update_exposure_range(imx06a);
	if (!ret)
		ret = __v4l2_ctrl_modify_range(imx06a->again,
					       IMX06A_ANA_GAIN_MIN,
					       mode->gain_max, mode->gain_step,
					       IMX06A_ANA_GAIN_MIN);
	return ret;
}

/*
 * Switch the advertised mode family. The active format moves to the nearest
 * mode of the new family, so the subdev never describes a mode it no longer
 * offers. Called from s_ctrl (control lock = state lock held); the HDR controls
 * are grabbed while streaming.
 */
static int imx06a_set_hdr(struct imx06a *imx06a, enum imx06a_hdr hdr)
{
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	const struct imx06a_mode *mode;

	if (hdr == imx06a->hdr)
		return 0;

	state = v4l2_subdev_get_locked_active_state(&imx06a->sd);
	fmt = v4l2_subdev_state_get_format(state, IMX06A_PAD_IMAGE);
	mode = imx06a_find_mode(hdr, imx06a_base_code(fmt->code), fmt->width,
				fmt->height);

	imx06a->hdr = hdr;
	imx06a->mode = mode;
	imx06a_state_set_mode(imx06a, state, mode);
	dev_dbg(imx06a->dev, "HDR mode %s -> %s\n", imx06a_hdr_menu[hdr],
		mode->name);
	return imx06a_set_framing_limits(imx06a);
}

static int imx06a_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx06a *imx06a =
		container_of(ctrl->handler, struct imx06a, ctrl_handler);
	struct v4l2_subdev_state *state;
	int ret = 0;

	/* Software side effects; the HDR controls have nothing to program. */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		ret = imx06a_update_exposure_range(imx06a);
		if (ret)
			return ret;
		break;
	case IMX06A_CID_HDR_MODE:
		ret = imx06a_set_hdr(imx06a, ctrl->val);
		/* Keep WIDE_DYNAMIC_RANGE in step (its handler no-ops on a match). */
		if (!ret && imx06a->wdr_ctrl->val != !!ctrl->val)
			ret = __v4l2_ctrl_s_ctrl(imx06a->wdr_ctrl, !!ctrl->val);
		return ret;
	case V4L2_CID_WIDE_DYNAMIC_RANGE:
		/* 1 -> DCG-HDR unless an HDR family is already selected; 0 -> off. */
		if (!!ctrl->val == !!imx06a->hdr)
			return 0;
		return __v4l2_ctrl_s_ctrl(imx06a->hdr_ctrl, ctrl->val ?
					  IMX06A_HDR_DCG : IMX06A_HDR_OFF);
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		/* Flips change the Bayer order of the active format. */
		state = v4l2_subdev_get_locked_active_state(&imx06a->sd);
		v4l2_subdev_state_get_format(state, IMX06A_PAD_IMAGE)->code =
			imx06a_current_code(imx06a, imx06a->mode->code);
		break;
	}

	if (pm_runtime_get_if_active(imx06a->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_VBLANK:
		ret = imx06a_write_timing(imx06a);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(imx06a->regmap, IMX06A_REG_ANALOG_GAIN,
				imx06a_hw_gain(imx06a->mode, ctrl->val), NULL);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		/* DCG-HDR requires DIG_GAIN == ST_DIG_GAIN; harmless elsewhere. */
		cci_write(imx06a->regmap, IMX06A_REG_GROUP_HOLD, 1, &ret);
		cci_write(imx06a->regmap, IMX06A_REG_DIGITAL_GAIN, ctrl->val, &ret);
		cci_write(imx06a->regmap, IMX06A_REG_ST_DIGITAL_GAIN, ctrl->val, &ret);
		cci_write(imx06a->regmap, IMX06A_REG_GROUP_HOLD, 0, &ret);
		break;
	case V4L2_CID_BRIGHTNESS:
		ret = imx06a_write_black_level(imx06a, ctrl->val);
		break;
	case V4L2_CID_HBLANK:
		ret = cci_write(imx06a->regmap, IMX06A_REG_LINE_LENGTH_INCK,
				imx06a_llinck(imx06a), NULL);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = cci_write(imx06a->regmap, IMX06A_REG_ORIENTATION,
				imx06a->hflip->val | (imx06a->vflip->val << 1),
				NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(imx06a->regmap, IMX06A_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN_RED ... V4L2_CID_TEST_PATTERN_GREENB:
		ret = cci_write(imx06a->regmap,
				IMX06A_REG_TEST_PATTERN_COLOUR(ctrl->id -
							       V4L2_CID_TEST_PATTERN_RED),
				ctrl->val, NULL);
		break;
	case V4L2_CID_NOTIFY_GAINS:
		ret = imx06a_write_wb_gains(imx06a);
		break;
	case IMX06A_CID_GPO_OUT:
	case IMX06A_CID_TESTOUT_OUT:
		/* In SW standby: these are grabbed while streaming. */
		ret = imx06a_write_sync_outputs(imx06a);
		break;
	default:
		break;
	}

	pm_runtime_mark_last_busy(imx06a->dev);
	pm_runtime_put_autosuspend(imx06a->dev);
	return ret;
}

static const struct v4l2_ctrl_ops imx06a_ctrl_ops = {
	.s_ctrl = imx06a_set_ctrl,
};

static const struct v4l2_ctrl_config imx06a_wb_gains_cfg = {
	.ops = &imx06a_ctrl_ops,
	.id = V4L2_CID_NOTIFY_GAINS,
	.type = V4L2_CTRL_TYPE_U32,
	.min = IMX06A_WB_GAIN_MIN,
	.max = IMX06A_WB_GAIN_MAX,
	.step = 1,
	.def = IMX06A_WB_GAIN_UNITY,
	.dims = { 4 },
};

static struct v4l2_ctrl *imx06a_new_menu(struct v4l2_ctrl_handler *hdl, u32 id,
					 const char *name,
					 const char * const *menu,
					 unsigned int n, u32 def)
{
	const struct v4l2_ctrl_config cfg = {
		.ops = &imx06a_ctrl_ops,
		.id = id,
		.name = name,
		.type = V4L2_CTRL_TYPE_MENU,
		.max = n - 1,
		.def = def,
		.qmenu = menu,
	};

	return v4l2_ctrl_new_custom(hdl, &cfg, NULL);
}

static int imx06a_init_controls(struct imx06a *imx06a)
{
	struct v4l2_ctrl_handler *hdl = &imx06a->ctrl_handler;
	const struct imx06a_mode *mode = imx06a->mode;
	u64 pixel_rate = (u64)imx06a->inck * IMX06A_PIX_PER_INCK;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int i, ret;

	/* 21 controls here, plus orientation and rotation from the fwnode. */
	ret = v4l2_ctrl_handler_init(hdl, 23);
	if (ret)
		return ret;

	ctrl = v4l2_ctrl_new_std(hdl, &imx06a_ctrl_ops, V4L2_CID_PIXEL_RATE,
				 pixel_rate, pixel_rate, 1, pixel_rate);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	ctrl = v4l2_ctrl_new_int_menu(hdl, &imx06a_ctrl_ops, V4L2_CID_LINK_FREQ,
				      0, 0, imx06a->link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Timing and gain ranges are set per mode by imx06a_set_framing_limits(). */
	imx06a->hblank = v4l2_ctrl_new_std(hdl, &imx06a_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0, 1, 0);
	imx06a->vblank = v4l2_ctrl_new_std(hdl, &imx06a_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0, 1, 0);
	imx06a->exposure = v4l2_ctrl_new_std(hdl, &imx06a_ctrl_ops,
					     V4L2_CID_EXPOSURE, mode->exp_min,
					     mode->exp_min + IMX06A_EXPOSURE_DEFAULT,
					     mode->exp_step,
					     mode->exp_min + IMX06A_EXPOSURE_DEFAULT);
	imx06a->again = v4l2_ctrl_new_std(hdl, &imx06a_ctrl_ops,
					  V4L2_CID_ANALOGUE_GAIN,
					  IMX06A_ANA_GAIN_MIN, mode->gain_max,
					  mode->gain_step, IMX06A_ANA_GAIN_MIN);
	v4l2_ctrl_new_std(hdl, &imx06a_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX06A_DGTL_GAIN_MIN, IMX06A_DGTL_GAIN_MAX, 1,
			  IMX06A_DGTL_GAIN_DEFAULT);
	v4l2_ctrl_new_std(hdl, &imx06a_ctrl_ops, V4L2_CID_BRIGHTNESS, 0,
			  IMX06A_DATA_PEDESTAL_MAX, 1, IMX06A_BLACK_LEVEL_DEFAULT);

	imx06a->hflip = v4l2_ctrl_new_std(hdl, &imx06a_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	imx06a->vflip = v4l2_ctrl_new_std(hdl, &imx06a_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu_items(hdl, &imx06a_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx06a_test_pattern_menu) - 1,
				     0, 0, imx06a_test_pattern_menu);
	for (i = 0; i < 4; i++)
		v4l2_ctrl_new_std(hdl, &imx06a_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i, 0,
				  IMX06A_TEST_PATTERN_COLOUR_MAX, 1,
				  IMX06A_TEST_PATTERN_COLOUR_MAX);

	imx06a->wb_gains = v4l2_ctrl_new_custom(hdl, &imx06a_wb_gains_cfg, NULL);

	imx06a->hdr_ctrl = imx06a_new_menu(hdl, IMX06A_CID_HDR_MODE, "HDR Mode",
					   imx06a_hdr_menu,
					   ARRAY_SIZE(imx06a_hdr_menu),
					   imx06a->hdr);
	imx06a->wdr_ctrl = v4l2_ctrl_new_std(hdl, &imx06a_ctrl_ops,
					     V4L2_CID_WIDE_DYNAMIC_RANGE, 0, 1, 1,
					     imx06a->hdr != IMX06A_HDR_OFF);
	imx06a->gpo_ctrl = imx06a_new_menu(hdl, IMX06A_CID_GPO_OUT, "GPO Output",
					   imx06a_strobe_menu,
					   ARRAY_SIZE(imx06a_strobe_menu),
					   imx06a->gpo_def);
	imx06a->testout_ctrl = imx06a_new_menu(hdl, IMX06A_CID_TESTOUT_OUT,
					       "TESTOUT Output",
					       imx06a_strobe_menu,
					       ARRAY_SIZE(imx06a_strobe_menu),
					       imx06a->testout_def);

	ret = v4l2_fwnode_device_parse(imx06a->dev, &props);
	if (!ret)
		ret = v4l2_ctrl_new_fwnode_properties(hdl, &imx06a_ctrl_ops, &props);
	if (!ret)
		ret = hdl->error;
	if (ret) {
		dev_err_probe(imx06a->dev, ret, "control init failed\n");
		goto err;
	}

	/* Flips change the Bayer order and HDR the mode list. */
	imx06a->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	imx06a->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	imx06a->hdr_ctrl->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	mutex_lock(hdl->lock);
	ret = imx06a_set_framing_limits(imx06a);
	mutex_unlock(hdl->lock);
	if (ret) {
		dev_err_probe(imx06a->dev, ret, "cannot set the timing limits\n");
		goto err;
	}

	imx06a->sd.ctrl_handler = hdl;
	return 0;

err:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

/* --------------------------------------------------------------------------
 * Runtime lane-rate selection (link_mbps)
 * --------------------------------------------------------------------------
 *
 * Writing link_mbps (Mbps, rounded to the nearest exact PLL rate, or a preset
 * name) re-synthesises the clocks, updates LINK_FREQ in place (the receiver
 * reads it at the next stream start) and re-ranges HBLANK. Refused while
 * streaming. Before a device is bound (module load) it only records the rate,
 * which then overrides the device tree at probe.
 */
static int imx06a_param_set_link_mbps(const char *val,
				      const struct kernel_param *kp)
{
	struct v4l2_subdev_state *state;
	struct imx06a *imx06a;
	unsigned int mbps;
	u64 req_hz, old_hz;
	char buf[16];
	int ret = 0;

	mutex_lock(&imx06a_self_lock);
	imx06a = imx06a_self;
	if (!kstrtouint(val, 0, &mbps)) {
		req_hz = (u64)mbps * HZ_PER_MHZ;
	} else if (imx06a) {
		/* A preset name: needs the device's INCK and PD setting. */
		strscpy(buf, val, sizeof(buf));
		ret = imx06a_preset_lane_hz(imx06a->dev, imx06a->inck,
					    imx06a->pdaf, strim(buf), &req_hz);
	} else {
		ret = -EINVAL;
	}
	if (ret)
		goto out;

	if (!imx06a) {
		if (mbps < IMX06A_LINK_MBPS_MIN || mbps > IMX06A_LINK_MBPS_MAX) {
			ret = -ERANGE;
		} else {
			link_param_hz = req_hz;
			link_mbps = mbps;
		}
		goto out;
	}

	state = v4l2_subdev_lock_and_get_active_state(&imx06a->sd);
	if (v4l2_subdev_is_streaming(&imx06a->sd)) {
		ret = -EBUSY;
	} else {
		old_hz = imx06a->link_req_hz;
		imx06a->link_req_hz = req_hz;
		ret = imx06a_update_clocks(imx06a);
		if (ret) {
			imx06a->link_req_hz = old_hz;
			imx06a_update_clocks(imx06a);
		} else {
			dev_info(imx06a->dev, "lane rate %llu kbps, %s line >= %u INCK\n",
				 div_u64(imx06a->clk.lane_hz, 1000),
				 imx06a->mode->name, imx06a->clk.llinck_min);
		}
	}
	v4l2_subdev_unlock_state(state);
out:
	mutex_unlock(&imx06a_self_lock);
	return ret;
}

static const struct kernel_param_ops imx06a_link_mbps_ops = {
	.set = imx06a_param_set_link_mbps,
	.get = param_get_uint,
};
module_param_cb(link_mbps, &imx06a_link_mbps_ops, &link_mbps, 0644);
MODULE_PARM_DESC(link_mbps,
		 "Lane rate in Mbps (300..2500) or a preset name, writable between streams; reads back the actual rate");

/* --------------------------------------------------------------------------
 * Pad ops
 * --------------------------------------------------------------------------
 */
static unsigned int imx06a_num_pads(const struct imx06a *imx06a)
{
	return imx06a->pdaf ? IMX06A_NUM_PADS : 1;
}

static int imx06a_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	static const u32 bases[] = {
		MEDIA_BUS_FMT_SRGGB10_1X10, MEDIA_BUS_FMT_SRGGB12_1X12,
		MEDIA_BUS_FMT_SRGGB14_1X14,
	};
	struct imx06a *imx06a = to_imx06a(sd);
	unsigned int i, j, n = 0;

	if (code->pad == IMX06A_PAD_PD) {
		if (code->index)
			return -EINVAL;
		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(bases); i++) {
		for (j = 0; j < ARRAY_SIZE(imx06a_modes); j++)
			if (imx06a_modes[j].hdr == imx06a->hdr &&
			    imx06a_modes[j].code == bases[i])
				break;
		if (j == ARRAY_SIZE(imx06a_modes))
			continue;
		if (n++ == code->index) {
			code->code = imx06a_current_code(imx06a, bases[i]);
			return 0;
		}
	}
	return -EINVAL;
}

static int imx06a_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx06a *imx06a = to_imx06a(sd);
	unsigned int i, n = 0, bpp;
	u32 base;

	if (fse->pad == IMX06A_PAD_PD) {
		if (fse->index || fse->code != MEDIA_BUS_FMT_SENSOR_DATA)
			return -EINVAL;
		fse->min_width = IMX06A_PD_PAD_WIDTH;
		fse->max_width = IMX06A_PD_PAD_WIDTH;
		fse->min_height = 1;
		fse->max_height = 1;
		return 0;
	}

	if (!imx06a_code_bpp(fse->code, &bpp))
		return -EINVAL;
	base = imx06a_base_code(fse->code);

	for (i = 0; i < ARRAY_SIZE(imx06a_modes); i++) {
		const struct imx06a_mode *m = &imx06a_modes[i];

		if (m->code != base || m->hdr != imx06a->hdr)
			continue;
		if (n++ == fse->index) {
			fse->min_width = m->width;
			fse->max_width = m->width;
			fse->min_height = m->height;
			fse->max_height = m->height;
			return 0;
		}
	}
	return -EINVAL;
}

static int imx06a_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx06a *imx06a = to_imx06a(sd);
	const struct imx06a_mode *mode;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	/* The PD pad follows the image mode; its format is not negotiable. */
	if (fmt->pad == IMX06A_PAD_PD) {
		fmt->format = *v4l2_subdev_state_get_format(state, IMX06A_PAD_PD);
		return 0;
	}

	mode = imx06a_find_mode(imx06a->hdr, imx06a_base_code(fmt->format.code),
				fmt->format.width, fmt->format.height);
	imx06a_state_set_mode(imx06a, state, mode);
	fmt->format = *v4l2_subdev_state_get_format(state, IMX06A_PAD_IMAGE);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		imx06a->mode = mode;
		return imx06a_set_framing_limits(imx06a);
	}
	return 0;
}

static int imx06a_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	if (sel->pad != IMX06A_PAD_IMAGE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, IMX06A_PAD_IMAGE);
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX06A_NATIVE_WIDTH;
		sel->r.height = IMX06A_NATIVE_HEIGHT;
		return 0;
	}
	return -EINVAL;
}

static int imx06a_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	const struct imx06a_mode *mode = to_imx06a(sd)->mode;
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = IMX06A_PAD_IMAGE,
		.format = {
			.code = mode->code,
			.width = mode->width,
			.height = mode->height,
		},
	};

	return imx06a_set_pad_format(sd, state, &fmt);
}

static int imx06a_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct imx06a *imx06a = to_imx06a(sd);
	unsigned int bpp = imx06a_mode_bpp(imx06a->mode);

	if (pad >= imx06a_num_pads(imx06a))
		return -EINVAL;

	memset(fd, 0, sizeof(*fd));
	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;
	if (pad == IMX06A_PAD_PD) {
		fd->entry[0].bus.csi2.vc = pd_vc & IMX06A_CSI2_VC_MASK;
		fd->entry[0].bus.csi2.dt = pd_dt & IMX06A_CSI2_DT_MASK;
	} else {
		fd->entry[0].bus.csi2.dt = bpp == 14 ? MIPI_CSI2_DT_RAW14 :
					   bpp == 12 ? MIPI_CSI2_DT_RAW12 :
						       MIPI_CSI2_DT_RAW10;
	}
	return 0;
}

/* --------------------------------------------------------------------------
 * Streaming
 * --------------------------------------------------------------------------
 */
static void imx06a_pm_put(struct imx06a *imx06a)
{
	pm_runtime_mark_last_busy(imx06a->dev);
	pm_runtime_put_autosuspend(imx06a->dev);
}

static void imx06a_grab_ctrls(struct imx06a *imx06a, bool grab)
{
	struct v4l2_ctrl *ctrls[] = {
		imx06a->vflip, imx06a->hflip, imx06a->hdr_ctrl,
		imx06a->wdr_ctrl, imx06a->gpo_ctrl, imx06a->testout_ctrl,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ctrls); i++)
		__v4l2_ctrl_grab(ctrls[i], grab);
}

/* Global registers, once per power-on; EXCK_FREQ first (SRM 5.1.1.1). */
static void imx06a_write_globals(struct imx06a *imx06a, int *ret)
{
	cci_write(imx06a->regmap, IMX06A_REG_EXCK_FREQ,
		  div_u64((u64)imx06a->inck * 256, HZ_PER_MHZ), ret);
	cci_multi_reg_write(imx06a->regmap, imx06a_global_regs,
			    ARRAY_SIZE(imx06a_global_regs), ret);
}

/* PLLs and line length. The line period itself is the HBLANK control's. */
static void imx06a_write_clocks(struct imx06a *imx06a, int *ret)
{
	const struct imx06a_clocks *c = &imx06a->clk;

	cci_write(imx06a->regmap, IMX06A_REG_IVT_PXCK_DIV, IMX06A_IVT_PXCK_DIV, ret);
	cci_write(imx06a->regmap, IMX06A_REG_IVT_SYCK_DIV, IMX06A_IVT_SYCK_DIV, ret);
	cci_write(imx06a->regmap, IMX06A_REG_IVT_PREPLLCK_DIV, c->ivt_prediv, ret);
	cci_write(imx06a->regmap, IMX06A_REG_IVT_PLL_MPY, c->ivt_mpy, ret);
	cci_write(imx06a->regmap, IMX06A_REG_IOP_SYCK_DIV, c->iop_div, ret);
	cci_write(imx06a->regmap, IMX06A_REG_IOP_PREPLLCK_DIV, c->iop_prediv, ret);
	cci_write(imx06a->regmap, IMX06A_REG_IOP_PLL_MPY, c->iop_mpy, ret);
	cci_write(imx06a->regmap, IMX06A_REG_LINE_LENGTH_PCK, c->llpck, ret);
	cci_write(imx06a->regmap, IMX06A_REG_MC_FRM_LP_EN, 1, ret);
}

/*
 * Analogue window (rows only; the columns must stay 0..8191), digital crop and
 * output size, written for every mode.
 */
static void imx06a_write_geometry(struct imx06a *imx06a, int *ret)
{
	const struct imx06a_mode *m = imx06a->mode;

	cci_write(imx06a->regmap, IMX06A_REG_X_ADD_STA, 0, ret);
	cci_write(imx06a->regmap, IMX06A_REG_X_ADD_END, IMX06A_NATIVE_WIDTH - 1, ret);
	cci_write(imx06a->regmap, IMX06A_REG_Y_ADD_STA, m->y_start, ret);
	cci_write(imx06a->regmap, IMX06A_REG_Y_ADD_END, m->y_start + m->rows - 1, ret);
	cci_write(imx06a->regmap, IMX06A_REG_DIG_CROP_X_OFFSET, m->dig_x, ret);
	cci_write(imx06a->regmap, IMX06A_REG_DIG_CROP_Y_OFFSET, m->dig_y, ret);
	cci_write(imx06a->regmap, IMX06A_REG_DIG_CROP_WIDTH, m->width, ret);
	cci_write(imx06a->regmap, IMX06A_REG_DIG_CROP_HEIGHT, m->height, ret);
	cci_write(imx06a->regmap, IMX06A_REG_X_OUT_SIZE, m->width, ret);
	cci_write(imx06a->regmap, IMX06A_REG_Y_OUT_SIZE, m->height, ret);
}

static int imx06a_write_mode(struct imx06a *imx06a)
{
	const struct imx06a_mode *mode = imx06a->mode;
	struct imx06a_pd_geom pd;
	int ret = 0;

	if (!imx06a->globals_written) {
		imx06a_write_globals(imx06a, &ret);
		imx06a->globals_written = !ret;
	}
	cci_multi_reg_write(imx06a->regmap, mode->regs, mode->num_regs, &ret);
	imx06a_write_geometry(imx06a, &ret);
	imx06a_write_clocks(imx06a, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_FAST_STANDBY, 0, &ret);
	/*
	 * D-PHY skew calibration off: the RP1 receiver does not use the deskew
	 * bursts the sensor sends above 1.5 Gbps and loses SoT sync on them.
	 */
	cci_write(imx06a->regmap, IMX06A_REG_SCAL_PERIOD_EN, 0, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_SCAL_INIT_EN, 0, &ret);
	/* PD output: all-pixel or sparse, see imx06a_pd_geom(). */
	imx06a_pd_geom(mode, &pd);
	cci_write(imx06a->regmap, IMX06A_REG_PHASE_PIX,
		  imx06a->pdaf && pd.phase_pix, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_PDAF_TYPE2,
		  imx06a->pdaf && !pd.phase_pix, &ret);
	return ret;
}

static int imx06a_apply_dbg_regs(struct imx06a *imx06a)
{
	char *buf, *p, *tok;
	int ret = 0;

	if (!dbg_regs || !*dbg_regs)
		return 0;
	buf = kstrdup(dbg_regs, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	p = buf;
	while ((tok = strsep(&p, ",")) && !ret) {
		char *eq = strchr(tok, '=');
		unsigned int a, v;

		if (!eq)
			continue;
		*eq = '\0';
		if (kstrtouint(tok, 0, &a) || kstrtouint(eq + 1, 0, &v))
			continue;
		dev_info(imx06a->dev, "dbg write 0x%04x = 0x%02x\n", a, v & 0xff);
		ret = cci_write(imx06a->regmap, CCI_REG8(a), v & 0xff, NULL);
	}
	kfree(buf);
	return ret;
}

static void imx06a_log_stream(struct imx06a *imx06a)
{
	const struct imx06a_mode *mode = imx06a->mode;
	struct imx06a_pd_geom pd;

	dev_dbg(imx06a->dev,
		"streaming %s: %llu kbps/lane, IVTCK %llu MHz, LLPCK %u, LLINCK %u, FLL %u\n",
		mode->name, div_u64(imx06a->clk.lane_hz, 1000),
		div_u64(imx06a->clk.ivtck, HZ_PER_MHZ), imx06a->clk.llpck,
		imx06a_llinck(imx06a), imx06a_fll(imx06a));
	if (!imx06a->pdaf)
		return;
	imx06a_pd_geom(mode, &pd);
	dev_dbg(imx06a->dev,
		"PD output: %s, %u values x %u rows RAW10 (%u B/frame), VC %u DT 0x%02x\n",
		pd.phase_pix ? "all-pixel" : "sparse", pd.px, pd.rows,
		pd.px * IMX06A_PD_BPP / 8 * pd.rows, pd_vc & IMX06A_CSI2_VC_MASK,
		pd_dt & IMX06A_CSI2_DT_MASK);
}

static int imx06a_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx06a *imx06a = to_imx06a(sd);
	int ret;

	/* PD rides the image's CSI-2 stream: only the image pad drives the sensor. */
	if (pad != IMX06A_PAD_IMAGE)
		return 0;

	ret = pm_runtime_resume_and_get(imx06a->dev);
	if (ret < 0)
		return ret;

	ret = imx06a_write_mode(imx06a);
	if (ret) {
		dev_err(imx06a->dev, "failed to write mode registers (%d)\n", ret);
		goto err_pm_put;
	}

	ret = __v4l2_ctrl_handler_setup(imx06a->sd.ctrl_handler);
	if (ret) {
		dev_err(imx06a->dev, "failed to apply controls (%d)\n", ret);
		goto err_pm_put;
	}

	/* Bring-up overrides go last, so they win over tables and controls. */
	ret = imx06a_apply_dbg_regs(imx06a);
	if (!ret)
		ret = cci_write(imx06a->regmap, IMX06A_REG_MODE_SELECT,
				IMX06A_MODE_STREAMING, NULL);
	if (ret)
		goto err_pm_put;

	imx06a_grab_ctrls(imx06a, true);
	imx06a_log_stream(imx06a);
	return 0;

err_pm_put:
	imx06a_pm_put(imx06a);
	return ret;
}

/*
 * Registers for the next session may only be written after the last frame's
 * FE (SRM 7.1.2). FRM_CNT reads 0xFF once the sensor is in SW standby, so wait
 * for that rather than guessing a delay.
 */
static int imx06a_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx06a *imx06a = to_imx06a(sd);
	u64 frame_us, cnt = 0;
	int ret = 0, err;
	bool fast;

	if (pad != IMX06A_PAD_IMAGE)
		return 0;

	/*
	 * Normal standby finishes the frame in flight, which for a long exposure
	 * can take tens of seconds. Above 1 s use fast standby, which aborts it.
	 */
	frame_us = div_u64((u64)imx06a_llinck(imx06a) * imx06a_fll(imx06a) *
			   USEC_PER_SEC, imx06a->inck);
	fast = frame_us > USEC_PER_SEC;
	if (fast)
		cci_write(imx06a->regmap, IMX06A_REG_FAST_STANDBY, 1, &ret);
	cci_write(imx06a->regmap, IMX06A_REG_MODE_SELECT, IMX06A_MODE_STANDBY,
		  &ret);
	if (ret)
		dev_err(imx06a->dev, "failed to stop streaming (%d)\n", ret);

	/* Two frames (or 0.2 s when fast) plus slack. */
	err = read_poll_timeout(cci_read, ret,
				ret || cnt == IMX06A_FRM_CNT_STANDBY,
				2000, (fast ? 0 : 2 * frame_us) + 200000, false,
				imx06a->regmap, IMX06A_REG_FRM_CNT, &cnt, NULL);
	if (err || ret)
		dev_warn(imx06a->dev,
			 "sensor did not reach SW standby (frm_cnt 0x%02llx, %d/%d)\n",
			 cnt, err, ret);

	imx06a_grab_ctrls(imx06a, false);
	imx06a_pm_put(imx06a);
	return 0;
}

/*
 * v4l2_subdev_s_stream_helper() only handles a single source pad. With the PD
 * pad present, the legacy s_stream (used by the RP1 CFE) drives the image pad.
 */
static int imx06a_s_stream(struct v4l2_subdev *sd, int enable)
{
	if (!to_imx06a(sd)->pdaf)
		return v4l2_subdev_s_stream_helper(sd, enable);

	/* These take the state lock themselves. */
	return enable ?
	       v4l2_subdev_enable_streams(sd, IMX06A_PAD_IMAGE, BIT_ULL(0)) :
	       v4l2_subdev_disable_streams(sd, IMX06A_PAD_IMAGE, BIT_ULL(0));
}

/* --------------------------------------------------------------------------
 * Power
 * --------------------------------------------------------------------------
 */
static int imx06a_power_on(struct device *dev)
{
	struct imx06a *imx06a = to_imx06a(dev_get_drvdata(dev));
	int ret;

	ret = regulator_bulk_enable(IMX06A_NUM_SUPPLIES, imx06a->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators (%d)\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(imx06a->xclk);
	if (ret) {
		dev_err(dev, "failed to enable clock (%d)\n", ret);
		regulator_bulk_disable(IMX06A_NUM_SUPPLIES, imx06a->supplies);
		return ret;
	}

	gpiod_set_value_cansleep(imx06a->reset_gpio, 0);
	usleep_range(IMX06A_POWER_ON_DELAY_US, IMX06A_POWER_ON_DELAY_US + 1000);
	imx06a->globals_written = false;
	return 0;
}

static int imx06a_power_off(struct device *dev)
{
	struct imx06a *imx06a = to_imx06a(dev_get_drvdata(dev));

	gpiod_set_value_cansleep(imx06a->reset_gpio, 1);
	clk_disable_unprepare(imx06a->xclk);
	regulator_bulk_disable(IMX06A_NUM_SUPPLIES, imx06a->supplies);
	return 0;
}

/* --------------------------------------------------------------------------
 * Subdev ops
 * --------------------------------------------------------------------------
 */
static const struct v4l2_subdev_core_ops imx06a_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx06a_video_ops = {
	.s_stream = imx06a_s_stream,
};

static const struct v4l2_subdev_pad_ops imx06a_pad_ops = {
	.enum_mbus_code = imx06a_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx06a_set_pad_format,
	.get_selection = imx06a_get_selection,
	.enum_frame_size = imx06a_enum_frame_size,
	.get_frame_desc = imx06a_get_frame_desc,
	.enable_streams = imx06a_enable_streams,
	.disable_streams = imx06a_disable_streams,
};

static const struct v4l2_subdev_ops imx06a_subdev_ops = {
	.core = &imx06a_core_ops,
	.video = &imx06a_video_ops,
	.pad = &imx06a_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx06a_internal_ops = {
	.init_state = imx06a_init_state,
};

/* --------------------------------------------------------------------------
 * Probe
 * --------------------------------------------------------------------------
 */
static int imx06a_check_hwcfg(struct imx06a *imx06a)
{
	struct device *dev = imx06a->dev;
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *endpoint;
	int ret = 0;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint)
		return dev_err_probe(dev, -EINVAL, "endpoint node not found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret)
		return dev_err_probe(dev, ret, "could not parse endpoint\n");

	if (ep.bus.mipi_csi2.num_data_lanes != 4)
		ret = dev_err_probe(dev, -EINVAL,
				    "only 4 data lanes are supported (got %u)\n",
				    ep.bus.mipi_csi2.num_data_lanes);
	else if (!ep.nr_of_link_frequencies)
		ret = dev_err_probe(dev, -EINVAL, "no link-frequencies in endpoint\n");
	else	/* DDR: lane rate = 2 x link frequency */
		imx06a->link_req_hz = 2 * ep.link_frequencies[0];

	v4l2_fwnode_endpoint_free(&ep);
	return ret;
}

static u32 imx06a_read_u32_max(struct device *dev, const char *prop, u32 max)
{
	u32 val;

	if (device_property_read_u32(dev, prop, &val) || val > max)
		return 0;
	return val;
}

/* Overlay parameters (see imx06a-overlay.dts). Returns a link preset name. */
static const char *imx06a_parse_properties(struct imx06a *imx06a)
{
	struct device *dev = imx06a->dev;
	const char *str;
	u64 hz;

	imx06a->pdaf = pdaf || device_property_read_bool(dev, "sony,pdaf");
	imx06a->gpo_def = imx06a_read_u32_max(dev, "sony,gpo", IMX06A_STROBE_HSYNC);
	imx06a->testout_def = imx06a_read_u32_max(dev, "sony,testout",
						  IMX06A_STROBE_HSYNC);
	imx06a->hdr = imx06a_read_u32_max(dev, "sony,hdr-mode", IMX06A_HDR_QBC);
	imx06a->mode = imx06a_default_mode(imx06a->hdr);

	/* "link-frequency=": a link frequency in Hz or a preset name. */
	if (device_property_read_string(dev, "sony,link-frequency", &str))
		return NULL;
	if (kstrtou64(str, 0, &hz))
		return str;
	imx06a->link_req_hz = 2 * hz;
	return NULL;
}

static int imx06a_init_link(struct imx06a *imx06a, const char *preset)
{
	int ret;

	if (link_param_hz) {
		imx06a->link_req_hz = link_param_hz;
	} else if (preset) {
		ret = imx06a_preset_lane_hz(imx06a->dev, imx06a->inck, imx06a->pdaf,
					    preset, &imx06a->link_req_hz);
		if (ret)
			return ret;
	}
	ret = imx06a_synth_clocks(imx06a->inck, imx06a->link_req_hz, imx06a->mode,
				  imx06a->pdaf, &imx06a->clk);
	if (ret)
		return dev_err_probe(imx06a->dev, ret,
				     "cannot synthesise a %llu Hz lane rate from a %u Hz INCK\n",
				     imx06a->link_req_hz, imx06a->inck);
	return 0;
}

static int imx06a_identify(struct imx06a *imx06a, u64 *rev)
{
	u64 val = 0;
	int ret, tries;

	for (tries = 0; tries < 5; tries++) {
		ret = cci_read(imx06a->regmap, IMX06A_REG_CHIP_ID, &val, NULL);
		if (!ret)
			break;
		usleep_range(10000, 11000);
	}
	if (ret)
		return dev_err_probe(imx06a->dev, ret, "failed to read chip id\n");
	if (val != IMX06A_CHIP_ID)
		return dev_err_probe(imx06a->dev, -ENODEV,
				     "unexpected chip id 0x%04llx (expected 0x%04x)\n",
				     val, IMX06A_CHIP_ID);
	return cci_read(imx06a->regmap, IMX06A_REG_REVISION, rev, NULL);
}

static int imx06a_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx06a *imx06a;
	const char *preset;
	unsigned int i;
	u64 rev = 0;
	int ret;

	imx06a = devm_kzalloc(dev, sizeof(*imx06a), GFP_KERNEL);
	if (!imx06a)
		return -ENOMEM;
	imx06a->dev = dev;

	v4l2_i2c_subdev_init(&imx06a->sd, client, &imx06a_subdev_ops);

	ret = imx06a_check_hwcfg(imx06a);
	if (ret)
		return ret;
	preset = imx06a_parse_properties(imx06a);

	imx06a->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx06a->regmap))
		return dev_err_probe(dev, PTR_ERR(imx06a->regmap),
				     "failed to init CCI regmap\n");

	imx06a->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx06a->xclk))
		return dev_err_probe(dev, PTR_ERR(imx06a->xclk),
				     "failed to get xclk\n");
	imx06a->inck = clk_get_rate(imx06a->xclk);

	for (i = 0; i < IMX06A_NUM_SUPPLIES; i++)
		imx06a->supplies[i].supply = imx06a_supply_name[i];
	ret = devm_regulator_bulk_get(dev, IMX06A_NUM_SUPPLIES, imx06a->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	imx06a->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(imx06a->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(imx06a->reset_gpio),
				     "failed to get reset gpio\n");

	ret = imx06a_init_link(imx06a, preset);
	if (ret)
		return ret;

	ret = imx06a_power_on(dev);
	if (ret)
		return ret;

	ret = imx06a_identify(imx06a, &rev);
	if (ret && !probe_stay_on)
		goto err_power_off;

	/* With runtime PM still disabled, setting up the controls writes nothing. */
	ret = imx06a_init_controls(imx06a);
	if (ret)
		goto err_power_off;

	imx06a->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	imx06a->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx06a->sd.internal_ops = &imx06a_internal_ops;
	imx06a->pads[IMX06A_PAD_IMAGE].flags = MEDIA_PAD_FL_SOURCE;
	imx06a->pads[IMX06A_PAD_PD].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx06a->sd.entity, imx06a_num_pads(imx06a),
				     imx06a->pads);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init entity pads\n");
		goto err_ctrls;
	}

	imx06a->sd.state_lock = imx06a->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&imx06a->sd);
	if (ret) {
		dev_err_probe(dev, ret, "subdev init error\n");
		goto err_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, IMX06A_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(dev);

	ret = v4l2_async_register_subdev_sensor(&imx06a->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register sensor subdev\n");
		goto err_pm;
	}

	mutex_lock(&imx06a_self_lock);
	if (imx06a_self)
		dev_warn(dev, "link_mbps stays bound to the first IMX06A\n");
	else
		imx06a_self = imx06a;
	mutex_unlock(&imx06a_self_lock);

	dev_info(dev, "IMX06A rev 0x%02llx: INCK %u Hz, lane %llu kbps%s%s%s\n",
		 rev, imx06a->inck, div_u64(imx06a->clk.lane_hz, 1000),
		 preset ? " (preset " : "", preset ?: "", preset ? ")" : "");

	if (!probe_stay_on)
		imx06a_pm_put(imx06a);
	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&imx06a->sd);
err_entity:
	media_entity_cleanup(&imx06a->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&imx06a->ctrl_handler);
err_power_off:
	imx06a_power_off(dev);
	return ret;
}

static void imx06a_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx06a *imx06a = to_imx06a(sd);

	mutex_lock(&imx06a_self_lock);
	if (imx06a_self == imx06a)
		imx06a_self = NULL;
	mutex_unlock(&imx06a_self_lock);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&imx06a->ctrl_handler);

	pm_runtime_disable(imx06a->dev);
	if (probe_stay_on)
		pm_runtime_put_noidle(imx06a->dev);
	if (!pm_runtime_status_suspended(imx06a->dev))
		imx06a_power_off(imx06a->dev);
	pm_runtime_set_suspended(imx06a->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx06a_pm_ops, imx06a_power_off,
				 imx06a_power_on, NULL);

static const struct of_device_id imx06a_of_match[] = {
	{ .compatible = "sony,imx06a" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx06a_of_match);

static struct i2c_driver imx06a_i2c_driver = {
	.driver = {
		.name = "imx06a",
		.of_match_table = imx06a_of_match,
		.pm = pm_ptr(&imx06a_pm_ops),
	},
	.probe = imx06a_probe,
	.remove = imx06a_remove,
};
module_i2c_driver(imx06a_i2c_driver);

MODULE_AUTHOR("Will Whang <will127534@gmail.com>");
MODULE_DESCRIPTION("Sony IMX06A sensor driver");
MODULE_LICENSE("GPL");
