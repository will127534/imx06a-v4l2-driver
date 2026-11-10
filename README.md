# Sony IMX06A V4L2 driver (Raspberry Pi 5)

<img width="3000" height="2000" alt="_DSC4677" src="https://github.com/user-attachments/assets/0f3b807b-98bc-437e-bb7f-1b4425ac1e52" />
<img width="3000" height="2000" alt="_DSC4656" src="https://github.com/user-attachments/assets/64c45c33-62cc-4789-9331-59b4217fa119" />
Driver for the Sony IMX06A-AJ1R-J: 50.3 MP (8192×6144), Type 1/0.98, 1.6 µm,
stacked BSI, Quad-Bayer colour filter with on-chip QBC remosaic, CCS/SMIA++-style
register map. 4-lane MIPI D-PHY.

## Modes

Maximum fps at the default **1752 Mbps/lane**, and at `link-frequency=MAX`
(2112 Mbps, the fastest the RP1 takes error-free; see Link speed). "Sensor" is what
the sensor delivers (raw CFE capture); "libcamera" is what rpicam/picamera2 get once
libcamera lengthens the line for the PiSP front end (~647 Mpix/s at the 333 MHz RP1
clock). Every mode is verified with bit-exact colour bars; the 2112 Mbps figures with
150-frame live-scene soaks (zero D-PHY/packet/frame errors) and rpicam-vid with zero
drops.

| HDR Mode | Output | Readout | Sensor @1752 | libcamera @1752 | Sensor @2112 |
|---|---|---|---|---|---|
| Off | 8192×6144 RAW10 / RAW12 | full res, on-chip QBC remosaic | 12.56 / 10.58 | 12.39 / 10.58 | 15.01 / 12.69 |
| Off | 8192×4608 RAW10 / RAW12 | full res, 16:9 analogue crop | 16.55 / 13.95 | 16.33 / 13.95 | 19.78 / 16.72 |
| Off | 4096×3072 RAW10 | 4×2 adjacent-pixel binning, Low-Noise-2 ADC | 46.58 | 46.59 | 55.21 |
| Off | 4096×2304 RAW10 | binned, 16:9 analogue crop | 61.17 | 61.18 | 72.50 |
| Off | 3840×2160 RAW10 | binned, centred 4K UHD crop (7680×4320 of the array) | 68.39 | 68.39 | 81.31 |
| Off | 1920×1080 RAW10 | **4×4 binned**, centre crop (94 % width, 70 % height) | **235.29** | 235.29 | 273.97 |
| Off | 1280×720 RAW10 | **4×4 binned**, centre crop (62.5 % width, 47 % height) | **440.92** | 440.92 | 505.05 |
| DCG-HDR | 4096×3072 / 4096×2304 RAW12 | dual conversion gain 4:1, merged on chip | 39.02 / 50.98 | 39.02 / 50.98 | 39.86 / 52.09 |
| DCG-HDR | 4096×3072 / 4096×2304 RAW14 | dual conversion gain 16:1, merged on chip | 33.96 / 44.37 | 33.96 / 44.37 | 39.86 / 52.09 |
| QBC-HDR | 4096×3072 / 4096×2304 RAW12 | quad long/short (ratio 4), merged on chip | 39.40 / 51.64 | 39.40 / 51.64 | 46.88 / 61.45 |

**4×4 binned 1080p / 720p** (high frame rate): 4×4 binning with the plain-AD10 readout
and a half-length line (LLPCK 4416, a 31-INCK readout floor). At 2112 Mbps the
line is link-bound (73 INCK at 1080p, 55 at 720p; 85 / 63 at the 1752 default);
complete frames need ≥ 1192 / 856
lines (shorter frames lose their last rows), so the minimums are 1200 / 864 →
273.97 / 505.05 fps, bit-exact colour bars, through the ISP too (~568 / ~465 Mpix/s).
The 1080p window covers 7680×4320 of the array, replacing the earlier
narrow 1080p crop of the 4×2 binned readout. Frame length and exposure act in steps of 8 lines in this
mode (the libcamera helper's exact-rate search knows that): 720p pins 400/240/120/60
exactly (480 → 478.93), 1080p pins 120/60/30 exactly (240 → 239.87).

DCG-HDR reads each line at both conversion gains, so its line has a readout floor of
184 INCK cycles (7.67 µs) regardless of lane rate: below 182 the merged output is all
zeros (measured; highlights and mid-tones are identical from 184 to 300 at equal
exposure time).

Every mode outputs ordinary RGGB Bayer (flips change the phase; the flip
controls carry `V4L2_CTRL_FLAG_MODIFY_LAYOUT`). Black level is 4096 in 16-bit
units in every mode (64 @10 bit, 256 @12, 1024 @14; the HDR modes use the
sensor's pedestal-offset function to get there).

## HDR

Both HDR families are **single-frame, sensor-fused** outputs (like the IMX585's
Clear HDR), so they are plain linear Bayer streams to the CFE and libcamera:

* **DCG-HDR**: one exposure, read at two conversion gains and merged. The
  register gain is the high-side gain (≥12 dB at 4:1, ≥24 dB at 16:1), but the
  merged output is referenced to the low-side gain, so the driver exposes
  `V4L2_CID_ANALOGUE_GAIN` in **LSG units** and remaps to the register
  (`HSG = (16384 − 16384/ratio) + code/ratio`). AGC therefore sees a minimum of
  1.0× and the same sensitivity as the normal modes (measured within 2%).
* **QBC-HDR**: long and short (= long/4) exposures interleaved in each quad and
  merged. The long exposure fills the bottom quarter of the 12-bit range, so the
  mode is ¼ as sensitive; the libcamera helper reports that via
  `getModeSensitivity()` (lux and mode switches stay correct). AGC exposes about
  4× longer, buying roughly 2 stops of shadow SNR.

Select the family **before** starting libcamera (it enumerates formats once):

```bash
SD=$(media-ctl -d /dev/media3 -e "imx06a 11-001a")
v4l2-ctl -d $SD -c hdr_mode=1           # 0 off, 1 DCG-HDR, 2 QBC-HDR
                                        # (wide_dynamic_range=1 also selects DCG-HDR)
rpicam-still --mode 4096:3072:12 ...    # DCG 4:1 (or QBC-HDR when hdr_mode=2)
rpicam-still --mode 4096:3072:14:U ...  # DCG 16:1, RAW14: libcamera's SW stats path engages
```

or load the overlay with a default: `dtoverlay=imx06a,hdr=1`.

## Clocking and line timing

The sensor's D-PHY runs up to 4.5 Gbps/lane. On the RP1 receiver the
highest error-free rate measured is **2112 Mbps/lane**, provided the sensor's
D-PHY **skew calibration is off** (0x0830/0x0832 = 0, done by the driver at every
stream start; with the default deskew bursts every rate from 1536 Mbps throws SoT
errors and drops frames). From 2208 Mbps the receiver flags payload CRC errors
and real images get corrupted pixels (2304: ~120/Mpix), so 2112 is the ceiling.

* **Readout** — `LINE_LENGTH_PCK` may only take the per-mode values in SRM
  Table 4-6 (other values put the internal H-sync on interference → FPN). Every
  mode runs at its shortest legal LLPCK at the fastest IVTCK (~2.19 GHz).
* **Line period** — set separately by `LINE_LENGTH_INCK` (0x3740, INCK cycles),
  which governs frame timing once `MC_FRM_LP_EN` (0x306D) = 1: line time =
  LLINCK / INCK. Measured: frame rate follows LLINCK exactly, and row/column FPN
  and temporal noise are unchanged up to 2× the minimum line.
* **V4L2** — `PIXEL_RATE` = INCK × 64 (1536 MHz for 24 MHz) and `HBLANK` in steps
  of 64, so one step is one INCK cycle and `(width + HBLANK) / PIXEL_RATE` is the
  true line time. HBLANK is **extensible**; its minimum is the sensor/link floor:
  the readout time (`ceil(LLPCK·INCK / (1.6·IVTCK))`) or the CSI-2 transfer of one
  output line plus the measured per-line D-PHY overhead (500 ns + 500 UI), whichever
  is longer. A line shorter than that keeps the frame rate but the sensor silently
  drops the last rows of each frame, so the margin is measured, not guessed.
* **Receiver limits are libcamera's job** — the PiSP front-end pixel-rate limit is
  applied by libcamera lengthening HBLANK, using the live RP1 clock.

## Exact frame rates

With a single rate pinned (`--framerate N`), the libcamera helper does not just
round VBLANK at the minimum line (that lands within one line, ~0.01–0.1 %).
It searches the (line, frame length) lattice for `LLINCK × FLL == INCK / fps`
exactly, taking the shortest such line (least rolling-shutter skew) up to 1.5× the
minimum, the same approach as the imx294/imx492 helpers. rpicam passes the
period in truncated µs (24 fps → 41666 µs), so a request within 1 µs of an
integer rate that divides INCK aims at that rate's exact tick count. With 24 MHz,
24/25/30/48/50/60/100/120 fps are all whole tick counts. If no exact pair exists,
the closest one wins.

Measured from sensor timestamps (rpicam-vid `--save-pts`, ~8 s per point). Every
point reads −5…−10 ppm, which is the module crystal measured against the Pi
clock, the same at every rate; the register products are exact:

| Link | Mode | Pinned fps → measured |
|---|---|---|
| 2112 (MAX) | 4096×2304 | 24/25/30/50/60 → −5 ppm each |
| 2112 (MAX) | 3840×2160 | 24/25/30/50/60 → −6 ppm each |
| 2112 (MAX) | 1920×1080 | 30/60/100/120 → −6…−7 ppm |
| 2112 (MAX) | 4096×3072 | 24/25/30/48 → −3…−12 ppm |
| 2112 (MAX) | 8192×6144 | 5/10 → −6 ppm; 12 → −12 ppm* |
| each preset | its mode | 60 or 50 or 30, plus 24, 25 → −5…−10 ppm |

\* Full resolution at 12 fps has no exact pair at 2112 Mbps: the minimum FLL
(6370) caps the line at 313 INCK, and 2 000 000 has no divisor in 304–313. The
nearest pair is 306 × 6536 = 2 000 016 (−8 ppm).

## Controls

Exposure (lines; minimum, step and prohibited small values per SRM Table 5-25;
max = FLL − 64), analogue gain (`16384/(16384−code)`, code a multiple of 4; up to
24 dB at full res and 36 dB binned), digital gain, VBLANK, extensible HBLANK (line
length, see Clocking), H/V flip, test pattern
(solid / colour bars / fade-to-grey / PN9), and `V4L2_CID_NOTIFY_GAINS`, which
feeds libcamera's AWB gains to the on-chip remosaic (`ABS_GAIN_*`, written under
group hold).

**Black level** is `V4L2_CID_BRIGHTNESS` (as in the imx585 driver), in 12-bit LSBs
in every mode: 16-bit black = value × 16, default 256 = 4096 (64 @ 10 bit, 256 @ 12,
1024 @ 14). Normal modes write it to `MANUAL_DATA_PEDESTAL_VALUE` (0x302C, u10.2);
the HDR merge scales that register by ¼, so in HDR modes the driver splits the
target between it and `PEDESTAL_OFFSET` (0x32F0), which reproduces the stock HDR
setting at the default. Measured: 4096 / 8192 / 1600 at 256 / 512 / 100 in binned,
full-res, DCG-HDR RAW12/RAW14 and QBC-HDR. It can change mid-stream (takes effect
within a frame). The libcamera fork's software-stats path reads it as its BLC anchor
(scale = tuning 4096 / default 256 = 16); the ISP itself keeps the tuning's 4096.

## Long exposure

VBLANK and EXPOSURE accept values up to 512× the 16-bit register range. Beyond
65532 lines the driver programs `value >> n` with `CIT_LSHIFT = n` (SRM 11.3,
n ≤ 9), so exposure is quantised to 2^n lines there. The libcamera helper's
`getBlanking()` mirrors this. Frames longer than 1 s are stopped with the
documented fast SW-standby (0x0106), so a stream stop never waits out a
multi-second frame. Verified: 3 s raw frames (shift 3) and a 5 s
`rpicam-still --shutter 5000000` (ExposureTime 4.995 s, FrameDuration 5.0007 s).

## Link speed

The boot lane rate is the overlay parameter `link-frequency`, which takes either
a **preset name** or a **link frequency in Hz** (= Mbps / 2):

```
dtoverlay=imx06a                              # 1752 Mbps (default)
dtoverlay=imx06a,link-frequency=MAX           # 2112 Mbps, highest error-free
dtoverlay=imx06a,link-frequency=DCI60P        # 4096x2304 @ 60 -> 1752 Mbps
dtoverlay=imx06a,link-frequency=UHD60P        # 3840x2160 @ 60 -> 1644 Mbps
dtoverlay=imx06a,link-frequency=858000000     # any frequency, rounded
```

| Preset | Target | Lane rate | Max fps (min line) | Line at target (INCK) |
|---|---|---|---|---|
| `MAX` | highest error-free rate on the RP1 | 2112 | see Modes | — |
| `DCI60P` / `DCI50P` / `DCI30P` | 4096×2304 @ 60 / 50 / 30 | 1752 / 1440 / 840 | 61.17 / 51.25 / 30.68 | 160 / 192 / 320 |
| `UHD60P` / `UHD50P` / `UHD30P` | 3840×2160 @ 60 / 50 / 30 | 1644 / 1296 / 792 | 64.54 / 51.63 / 32.37 | 160 / 200 / 320 |

A preset is a (mode, frame rate) target that must be hit **exactly**: the driver
picks the longest line h (INCK cycles) that divides the target period
T = INCK / fps with T / h a legal frame length (multiple of 4, ≥ the mode's
minimum), then the lowest exact PLL rate whose link floor still fits that line
(same timing model as HBLANK, including the PD packets when `pdaf` is on:
DCI60P becomes 2232 Mbps, above the 2112 verified, and the driver warns). So the
preset rate is a little above the bare
"just reaches 60" rate, and libcamera then finds `h × FLL == T` with no residue
(DCI60P: 160 × 2500 = 400 000 = 24 MHz / 60). All six were verified with complete,
bit-exact colour bars at their minimum line. Names are
case-insensitive. A numeric value is rounded to the nearest exact PLL rate with
the PLL reference kept at the full INCK — with the 24 MHz module clock, the
**12 Mbps grid** (1080–2244 Mbps; 6/3 Mbps grids below via the IOP divider).
LINK_FREQ and `link_mbps` report the actual rate, and dmesg logs it on each change.

Rates can also be changed between streams (not while an app has the camera
open — libcamera reads the mode limits when it opens it):

```
echo 1600   | sudo tee /sys/module/imx06a/parameters/link_mbps   # -> 1596 (actual)
echo DCI60P | sudo tee /sys/module/imx06a/parameters/link_mbps   # presets work too
```

`link_mbps=` given at module load overrides the overlay at every probe.

| Lane rate | Full 4:3 R10 / R12 | Binned 4:3 | 4096×2304 | 4K UHD | 1080p |
|---|---|---|---|---|---|
| 1440 | 10.41 / 8.76 | 39.02 | 51.25 | 57.06 | 198.02 |
| 1728 | 12.39 / 10.44 | 46.01 | 60.42 | 67.50 | 232.56 |
| **1752** (default) | 12.39 / 10.58 | 46.59 | 61.18 | 68.39 | 235.29 |
| 2112 (MAX) | 12.39 / 12.39 | 49.03 | 64.39 | 72.46 | 273.97 |

(libcamera maxima at the 333 MHz RP1 clock; from ~1920 Mbps the wide modes are
held by the PiSP pixel-rate limit, raw capture is not.) Above 2112 Mbps the RP1
corrupts real images.

## Phase-detect (PD) output

Every pixel is a dual photodiode ("Dual PD"). With `pdaf` set, the sensor also
sends its phase data and the driver exposes it for an external focus system:

```
dtoverlay=imx06a,pdaf          # or: modprobe imx06a pdaf=1
```

* **On the wire**: `PHASE_PIX` (0x3104) = 1 makes the sensor interleave one PD
  packet after every 4 image lines, on **VC0, DT 0x30** (MIPI user-defined 0).
  Sony documents neither; the stream was found with the RP1 CSI-2 discard
  counters and decoded against the simultaneous image.
* **Layout**: always **RAW10**, CSI-2 packed, even under RAW12/14 HDR output.
  Each PD row holds **left/right photodiode values interleaved** (L0 R0 L1 R1 …).
  Each site covers 2 columns × 4 rows of the (binned-equivalent) image, and
  **L + R reproduces the image** (correlation 0.9998; black level 64).

  | Mode | PD per frame (values × rows) | Bytes/frame |
  |---|---|---|
  | binned 4:3, DCG-HDR, QBC-HDR | 4096 × 768 | 3 932 160 |
  | full 4:3 RAW10 | 4096 × 1536 | 7 864 320 |
  | full 16:9 RAW10 / binned 16:9 | 4096 × 1152 / 4096 × 576 | 5 898 240 / 2 949 120 |
  | 4K UHD / 1080p (follow the horizontal crop) | 3840 × 540 / 1920 × 270 | 2 592 000 / 648 000 |
  | 4×4 720p | 1280 × 180 | 288 000 |
  | full 4:3 / 16:9 **RAW12** | **sparse PD** 1016 × 3008 / 1016 × 2304 (below) | 3 820 160 / 2 926 080 |

  Full-res **RAW12** runs the non-All-PD ADC mode and instead carries a
  sparse "partial PD / LR sensitivity" pattern on the same VC/DT, enabled by
  `PDAF_TYPE2` (0x3103) = 1:
  * two packets of 1016 values per 4 image lines;
  * one sample per 8 columns (image column 8j + 37) and per 2 rows, over array
    rows 64–6079 clipped to the readout window;
  * left/right alternate in **row pairs** (rows 4k, 4k+1 one side, 4k+2, 4k+3
    the other), and each value is a single PD pixel at about full-pixel level.

  It is emitted exactly when 0x3103 = 1 for that stream (the mode table writes
  0), so the driver sets it; PHASE_PIX has no effect in this mode.

  DCG-HDR's PD comes from one gain path and clips at 511 in highlights; QBC-HDR's
  tracks the image (correlation 0.9996). The HDR PD-output registers
  0x38D1/0x38D2 change nothing in these modes.
  Unlike the IMX708, the sensor does **not** compute phase on chip. Its embedded
  data (DT 0x12, two lines) is a plain register dump of 166 registers (frame
  count, exposure, frame length, gain, temperature), identical with either PD
  enable, so phase has to be computed from these pixels. The debug
  line `PD output: N values x M rows RAW10 (B B/frame)` reports each stream
  (dev_dbg, like the `streaming ...` timing line).
* **Link budget**: the PD packets share the line time, so with `pdaf` the HBLANK
  minimum includes them (+1 packet per 4 lines, sparse: +2), e.g. binned 4:3 at
  2112 Mbps 135 → 169 INCK (raw max ~55 → ~44 fps), full res 285 INCK, full res
  RAW12 297 → 321 INCK. Short of that, the sensor silently drops the PD tail rows. Every mode was
  verified with bit-exact colour bars at that minimum. Without `pdaf` nothing
  changes (single pad, 0x3104 = 0, same line lengths).
* **V4L2**: a second source pad (`MEDIA_BUS_FMT_SENSOR_DATA`) that the RP1 CFE
  links to its CSI-2 channel 1 "embedded" node; `get_frame_desc` reports DT 0x30.
  The pad reports the **true PD size** of the mode: width = bytes per packet,
  height = packets per frame (e.g. binned 4:3 5120 × 768). The sensor→CSI-2 link
  is immutable and validated on every stream start, so the consumer must set the
  CSI-2 metadata pads to the same size: libcamera does when its IMX06A helper
  uses the PD data (the `imx06a-pdaf` libcamera branch). A libcamera **without** PD
  support leaves those pads at their default and stream start fails (EPIPE);
  load the module with `pd_standin=1` to advertise the CFE's default 16384 × 1
  instead (the receiver then stores DT 0x30 data up to the meta buffer size its
  consumer sets), or load without `pdaf`.
* **Reading it**: capture the image node (`rp1-cfe-csi2_ch0`) and the embedded node
  together, with the embedded node's meta buffer set to the PD size above, then split
  the RAW10 rows into the L and R planes.
* **Never stream the image and embedded nodes from two processes** (e.g. two
  `v4l2-ctl`): concurrent STREAMONs start the sensor twice, and the CFE's error
  path oopses (`csi2_stop_channel`), leaving the box needing a reboot.
  Issue them sequentially from one thread, as libcamera does.

## Sync / strobe output

Two menu controls, `GPO Output` and `TESTOUT Output`, each with the same choices.
Their defaults come from the overlay parameters `gpo=<n>` and `testout=<n>`
since libcamera does not set custom controls. They're
applied in SW standby at every stream start (raw V4L2 or libcamera) and locked
while streaming. The pins are independent (H and V sync use separate monitor
slots), e.g. `gpo=1 testout=2` = exposure strobe on GPO, V sync on TESTOUT.

| n | Output | Measured (logic analyzer, GPO and TESTOUT) |
|---|---|---|
| 0 | off | idle low |
| 1 | **exposure strobe** (`FSTROBE_MODE` 0x3228 = 2, monitor 0x2C) | high from the first row's exposure start to the end of readout: binned 4:3 = **exposure + 3104 lines**, line-exact (400 / 1000 / 2000 / 2800-line exposures → 3504.0 / 4104.0 / 5103.9 / 5903.9 lines); period = frame period exactly |
| 2 | V sync pulse (`MNTTEST2` = 0x016) | ~0.15 µs active-low pulse per frame |
| 3 | H sync pulse (`MNTTEST1` = 0x015) | ~0.15 µs active-low pulse per line (7.05 µs apart at LLINCK 169) |

* **Exposure strobe for IMU / light sync.** The rising edge is the start of
  exposure, the falling edge the end of readout. It follows mid-stream exposure,
  gain and line-length changes frame by frame; under libcamera (fixed 10 ms
  shutter, 15 fps) it measured 15.000 Hz and 35.866 ms high, as predicted. It only
  has edges while exposure + readout < frame time. At 30 fps with AE's 24.7 ms
  exposure it stays high (no edges); use a lower frame rate, a shorter exposure,
  or the V sync pulse there.
* **Why the strobe can stay high.** It spans first-row exposure start to
  last-row readout end, i.e. exposure + readout (~3104 lines binned). At the
  default VBLANK the frame is 3220 lines, so any exposure over ~116 lines makes
  consecutive pulses overlap. That's normal rolling-shutter operation (the top
  rows of frame N+1 start integrating while the bottom of frame N is still
  being read), not a timing error, but the pin then has no edges.
* **IMU frame sync (LSM6DSO on the camera I²C bus, 11-0x6a).** Wire GPO to the
  IMU's INT2/DEN pin and use DEN edge-sensitive trigger mode (AN5192 4.8.1):
  one gyro sample/FIFO entry per rising edge (CTRL6_C = 0x80, CTRL9_XL = 0xE4 for
  the rising edge): 23.6 samples/s while streaming at 23.669 fps, 0 when idle.
  INT2 powers up as an output driven LOW. Configure the IMU (switching INT2 to
  input) before streaming with `gpo` enabled, after every IMU power-up, or the
  two outputs fight. If the IMU is on the camera rail, use `dtoverlay=imx06a,always-on`
  in config.txt (boot-time only) so the rail stays up between streams.
* **Pins.** GPO (ball L1) is verified. The FSTROBE ball (K1) never toggled in any
  configuration: long strobe, forced TESTOUT, V sync, 0x3228 alone, or CCS flash
  triggering (`FLASH_MODE_CAPABILITY` 0x1A02 = 0). The CCS
  flash block 0x0C12–0x0C1C exists but is unused. A register sweep with the
  analyzer (0x3000–30FF, 0x3220–323F, 0x4000–411F, one
  register at a time in standby, plus selector × IO-control combinations and
  streamed tests) moved only GPO. 0x3034/0x3035 are an undocumented writable
  select/value pair in the pin-control block, but neither touches K1. Two
  undocumented registers break the probe: 0x4004 locks further register writes,
  and 0x4047 stops I²C. Recover with a runtime-PM power cycle. So K1 looks
  unused on this die, unless the board net is open. TESTOUT (C1) is verified
  too: forced levels, and on stream the exposure strobe identical to GPO
  (23.669 Hz, 28.899 ms high at FLL 6000 / exposure 1000), with the same ~0.1 µs
  V/H sync pulses. All are 1.8 V CMOS outputs (GPO 4 mA, TESTOUT 2 mA); a 3.3 V input
  needs a level shifter. The sync pulses ring on the edges with a 3.3 V-threshold
  analyzer. XVS, MCSSYNC and OISEN are "not supported, do not connect".
* The "OIS pulse" monitor (0x8A) is a ~240 kHz internal toggle, not a frame
  signal.

## Module parameters

* `hdr_mode` (V4L2 control, see HDR above) and the overlay parameter `hdr=0|1|2`.
* `pdaf` (bool, read-only at runtime): expose the PD output on a second pad for
  every device (see Phase-detect output). Per device, the overlay's `pdaf` does
  the same. Pads are fixed at probe; an overlay reload without `pdaf` drops the
  PD pad again even with the module still loaded.
* `pd_standin` (bool, load-time): advertise the PD pad at a fixed 16384 × 1
  instead of the true size, for a libcamera without PD support.
* `pd_dt`, `pd_vc`: bring-up aid, the data type / virtual channel the PD pad
  asks the receiver to capture (default 0x30 / 0; e.g. `pd_dt=0x12` captures the
  embedded data instead).
* `link_mbps` (default: the overlay's link-frequency, 1752): lane rate in Mbps or
  a preset name, rounded to the nearest exact PLL rate, writable between streams;
  reads back the actual rate.

* `dbg_regs`: bring-up aid, `addr=val,...` 8-bit writes applied just before
  stream-on (after the controls). Load-time only: reload the module with it, then
  re-apply the overlay.
* `probe_stay_on`: bring-up aid, continue probe even if the chip-ID read fails.

## Install

```bash
./setup.sh                      # DKMS build + install, installs imx06a.dtbo
# /boot/firmware/config.txt:
dtoverlay=imx06a                # CAM1 (add ",cam0" for CAM0)
```

Overlay parameters: `cam0`, `rotation`, `orientation`, `always-on` (keeps the
camera rail on; `regulator-always-on` is only honoured at boot, so put it in
config.txt and reboot),
`clock-frequency` (module oscillator, default 24000000), `hdr` (default HDR family),
`link-frequency` (preset name or Hz, see Link speed), `pdaf` (PD output pad),
`gpo` / `testout` (sync output defaults 0-3, see Sync / strobe output).

libcamera support (cam helper, sensor properties, `imx06a.json` PiSP tuning)
lives in the `will127534/libcamera` fork. The tuning is **provisional**: it is
based on IMX477 with grey-world AWB and flat lens shading until a CTT
calibration is done.
