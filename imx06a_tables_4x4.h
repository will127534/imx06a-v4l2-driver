/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IMX06A 4x4-binned (2048-wide) readout, used for the high-frame-rate crops:
 * 4x4 binning with the plain AD10 readout timing that its half-length line
 * (LLPCK 4416) needs.
 */
#ifndef IMX06A_TABLES_4X4_H
#define IMX06A_TABLES_4X4_H

static const struct cci_reg_sequence imx06a_mode_bin4_regs[] = {
	/* MIPI output setting */
	{ CCI_REG8(0x0112), 0x0A },
	{ CCI_REG8(0x0113), 0x0A },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x3239), 0x00 },
	/* ROI Setting */
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x1F },
	{ CCI_REG8(0x0349), 0xFF },
	{ CCI_REG8(0x034A), 0x17 },
	{ CCI_REG8(0x034B), 0xFF },
	/* Mode Setting */
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x44 }, /* 4x4 binning */
	{ CCI_REG8(0x0902), 0x02 }, /* 4x4 binning */
	{ CCI_REG8(0x3140), 0x0A },
	{ CCI_REG8(0x3144), 0x00 }, /* AD10 readout */
	{ CCI_REG8(0x3148), 0x04 },
	{ CCI_REG8(0x31E0), 0x43 }, /* 4x4 binning */
	{ CCI_REG8(0x31E1), 0x43 }, /* 4x4 binning */
	{ CCI_REG8(0x3205), 0x01 },
	{ CCI_REG8(0x3206), 0x00 },
	/* Digital Crop & Scaling */
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040A), 0x00 },
	{ CCI_REG8(0x040B), 0x00 },
	{ CCI_REG8(0x040C), 0x10 },
	{ CCI_REG8(0x040D), 0x00 },
	{ CCI_REG8(0x040E), 0x0C },
	{ CCI_REG8(0x040F), 0x00 },
	/* Output Size Setting */
	{ CCI_REG8(0x034C), 0x10 },
	{ CCI_REG8(0x034D), 0x00 },
	{ CCI_REG8(0x034E), 0x0C },
	{ CCI_REG8(0x034F), 0x00 },
	/* Other Setting */
	{ CCI_REG8(0x3702), 0x03 },
	{ CCI_REG8(0x3703), 0x00 },
	{ CCI_REG8(0x3708), 0xFF },
	{ CCI_REG8(0x3709), 0xFF },
	{ CCI_REG8(0x370A), 0xFE }, /* AD10 readout */
	{ CCI_REG8(0x370B), 0xD2 }, /* AD10 readout */
	{ CCI_REG8(0x370C), 0xFF },
	{ CCI_REG8(0x370D), 0xFF },
	{ CCI_REG8(0x370E), 0xFE }, /* AD10 readout */
	{ CCI_REG8(0x370F), 0xD2 }, /* AD10 readout */
	{ CCI_REG8(0x3710), 0xFF },
	{ CCI_REG8(0x3711), 0xFF },
	{ CCI_REG8(0x3712), 0xDA }, /* AD10 readout */
	{ CCI_REG8(0x3713), 0x40 }, /* AD10 readout */
	{ CCI_REG8(0x3714), 0xFF },
	{ CCI_REG8(0x3715), 0xFF },
	{ CCI_REG8(0x3716), 0xDA }, /* AD10 readout */
	{ CCI_REG8(0x3717), 0x40 }, /* AD10 readout */
	{ CCI_REG8(0x3718), 0x00 },
	{ CCI_REG8(0x3719), 0x00 },
	{ CCI_REG8(0x371A), 0x00 },
	{ CCI_REG8(0x371B), 0x00 },
	{ CCI_REG8(0x371C), 0x01 },
	{ CCI_REG8(0x371D), 0xDA }, /* AD10 readout */
	{ CCI_REG8(0x371E), 0x40 }, /* AD10 readout */
	{ CCI_REG8(0x371F), 0x01 },
	{ CCI_REG8(0x3720), 0xDA }, /* AD10 readout */
	{ CCI_REG8(0x3721), 0x40 }, /* AD10 readout */
	{ CCI_REG8(0x3722), 0xF3 },
	{ CCI_REG8(0x3723), 0x39 },
	{ CCI_REG8(0x3724), 0x01 },
	{ CCI_REG8(0x3725), 0x97 },
	{ CCI_REG8(0x3726), 0xFF },
	{ CCI_REG8(0x3727), 0xFF },
	{ CCI_REG8(0x3728), 0x00 },
	{ CCI_REG8(0x3729), 0xBF },
	{ CCI_REG8(0x372A), 0x01 },
	{ CCI_REG8(0x372B), 0x5F },
	{ CCI_REG8(0x3790), 0x00 },
	{ CCI_REG8(0x3791), 0x00 },
	{ CCI_REG8(0x3794), 0x00 },
	{ CCI_REG8(0x3795), 0x00 },
	{ CCI_REG8(0x3900), 0x00 },
	{ CCI_REG8(0x3901), 0x12 }, /* 4x4 binning */
	{ CCI_REG8(0x3904), 0x00 },
	{ CCI_REG8(0x3905), 0x12 }, /* 4x4 binning */
	{ CCI_REG8(0x3918), 0x03 }, /* AD10 readout */
	{ CCI_REG8(0x3919), 0xD8 }, /* 4x4 binning */
	{ CCI_REG8(0x391A), 0x03 }, /* AD10 readout */
	{ CCI_REG8(0x391B), 0xD8 }, /* 4x4 binning */
	/* Integration Setting */
	{ CCI_REG8(0x3162), 0x01 },
	{ CCI_REG8(0x3163), 0xF4 },
	/* Gain Setting */
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x00 },
	{ CCI_REG8(0x0218), 0x01 },
	{ CCI_REG8(0x0219), 0x00 },
	{ CCI_REG8(0x3164), 0x00 },
	{ CCI_REG8(0x3165), 0x00 },
	{ CCI_REG8(0x3166), 0x01 },
	{ CCI_REG8(0x3167), 0x00 },
	/* HDR mode Setting */
	{ CCI_REG8(0x0220), 0x00 },
	{ CCI_REG8(0x0221), 0x00 },
	{ CCI_REG8(0x0222), 0x01 },
	{ CCI_REG8(0x3161), 0x00 },
	/* DCGHDR Setting */
	{ CCI_REG8(0x3170), 0x00 },
	{ CCI_REG8(0x3172), 0x04 },
	{ CCI_REG8(0x320B), 0x01 },
	/* PHASE PIX Setting */
	{ CCI_REG8(0x3104), 0x00 },
	/* HDR PHASE PIX Output Data Setting */
	{ CCI_REG8(0x38D1), 0x00 },
	/* PDAF TYPE2 Setting */
	{ CCI_REG8(0x3103), 0x00 },
	{ CCI_REG8(0x3422), 0x03 },
	{ CCI_REG8(0x3423), 0xF8 },
	/* DCGHDR PDAF TYPE2 Output Data Setting */
	{ CCI_REG8(0x38D2), 0x00 },
	/* DOL Setting */
	{ CCI_REG8(0x3180), 0x00 },
	{ CCI_REG8(0x3181), 0x00 },
};

#endif /* IMX06A_TABLES_4X4_H */
