# HP TouchPad Firmware Files Identification
**Location:** `/home/herrie/webos/touchpad-kernel/doctor305/nova-cust-image-topaz.rootfs/lib/firmware/`

---

## FIRMWARE FILES BY COMPONENT

### 1. A6 BATTERY CONTROLLERS (TI MSP430)
**Files:**
- `a6_firmware.txt.00` (49 KB) - A6_0 firmware (I2C address 0x31)
- `a6_firmware.txt.01` (31 KB) - A6_1 firmware (I2C address 0x32)

**Format:** TI-TXT hex format for MSP430 microcontroller
**Purpose:** Battery monitoring, power management, Touchstone inductive charging
**Load Method:** SBW (Spy-Bi-Wire) programming via GPIO bit-banging
**Mainline Status:** ✅ Driver supports firmware loading via ioctl interface
**Driver:** `drivers/misc/a6/` with character device interface
**Firmware Interface:**
- `A6_IOCTL_SET_FW_DATA` - Write firmware to A6 controller
- `A6_IOCTL_VERIFY_FW_DATA` - Verify programmed firmware
**Notes:** Contains embedded strings like "This device's maker has agreed to Palm's Peripheral Connection License terms & conditions."

---

### 2. TOUCHSCREEN (Cypress CY8CTMA395)
**Files:**
- `cy8ctma395.fw` (82 KB) - Main touchscreen firmware (binary format)
- `cy8ctma395.hex` (159 KB) - Main touchscreen firmware (Intel HEX format)
- `cy8ctma395_hssp_bridge.fw` (82 KB) - HSSP bridge firmware (binary)
- `cy8ctma395_hssp_bridge.hex` (159 KB) - HSSP bridge firmware (Intel HEX)
- `cy8ctma395.ver` (7 bytes) - Firmware version: **15.0.0**
- `cy8ctma375.hex` (36 KB) - Alternative/older touchscreen firmware

**Format:** Intel HEX and binary formats  
**Purpose:** Touchscreen controller firmware  
**Load Method:** SWD (Serial Wire Debug) programming via GPIO bit-banging  
**Mainline Status:** ✅ Driver present, firmware loading supported  
**Notes:** Multiple formats provided for different programming methods

---

### 3. WIFI (Atheros AR6003 - ath6kl)
**Directory:** `ath6k/hw2.1.1/`

**Files:**
- `athwlan.bin` (69 KB) - Main WiFi firmware
- `bdata.SD32.bin` (1.8 KB) - **Board data (WiFi-only model calibration)**
- `3g_bdata.SD32.bin` (1.8 KB) - **Board data (3G model calibration)**
- `data.patch.bin` (172 bytes) - Data patch file
- `otp.bin` (2.8 KB) - One-Time Programmable data
- `athtcmd_ram.bin` (11 KB) - Test command RAM firmware
- `dbglog_id.h` - Debug logging header
- `dbglog.h` - Debug logging definitions

**Hardware:** Atheros AR6003 (hw2.1.1)  
**Purpose:** WiFi 802.11a/b/g/n connectivity  
**Load Method:** Loaded by ath6kl driver at runtime  
**Mainline Status:** ✅ Fully supported by mainline ath6kl driver  
**Critical:** Different board data files for WiFi vs 3G variants!

**Firmware Path in Mainline:**
- `/lib/firmware/ath6k/AR6003/hw2.1.1/`
- Needs proper bdata file selection based on hardware variant

---

### 4. GPU (Adreno 220)
**Files:**
- `leia_pfp_470.fw` (1.2 KB) - **Used by Adreno 220** - PFP (Pre-Fetch Parser) firmware
- `leia_pm4_470.fw` (9.1 KB) - **Used by Adreno 220** - PM4 microcode
- `yamato_pfp.fw` (1.2 KB) - For Adreno 200/205 only (NOT used by TouchPad)
- `yamato_pm4.fw` (9.1 KB) - For Adreno 200/205 only (NOT used by TouchPad)

**Purpose:** GPU microcode for Adreno 220 (a2xx family, "Leia" generation)
**Load Method:** Loaded by freedreno/msm driver
**Mainline Status:** ✅ Supported - mainline uses same firmware files

**Chip ID Detection:**
- Legacy kernel: `KGSL_CHIPID_LEIA_REV470` (0x2010000) selects leia_*_470.fw
- Mainline: `chip_ids = 0x02020000` for A220 selects leia_*_470.fw

**Mainline Firmware Path:**
- `/lib/firmware/qcom/leia_pfp_470.fw` ⭐ (required for TouchPad)
- `/lib/firmware/qcom/leia_pm4_470.fw` ⭐ (required for TouchPad)

**Note:** The "Yamato" firmware is for older Adreno 200/205 GPUs (A200 chip ID 0x02000000).
The HP TouchPad uses Adreno 220 which requires the "Leia" firmware.

### ⚠️ CRITICAL: TouchPad Firmware vs linux-firmware Differences

The TouchPad firmware files are **SIGNIFICANTLY DIFFERENT** from the linux-firmware versions:

| File | Same Size | Bytes Different | % Different |
|------|-----------|-----------------|-------------|
| leia_pfp_470.fw | 1156 bytes | 644 bytes | **56%** |
| leia_pm4_470.fw | 9220 bytes | 3335 bytes | **36%** |

**TouchPad firmware checksums:**
- `leia_pfp_470.fw`: `b045ebdaf1e3fa4ecfa06f677e9fa6a1`
- `leia_pm4_470.fw`: `2a81fc15f3651c763b0bfec95d6b4cee`

**linux-firmware checksums (from April 2022, Dmitry Baryshkov):**
- `leia_pfp_470.fw`: `73697be0acbc0093cda74d4987cf7ab1`
- `leia_pm4_470.fw`: `dbc0f42b3dce0ae3fcb558b737fb073f`

**Implications:**
1. Palm/HP likely used **custom GPU microcode** for webOS
2. The linux-firmware files are from a different MSM8x60 device/source
3. This may explain why [freedreno has issues on TouchPad](https://wiki.postmarketos.org/wiki/HP_TouchPad_(hp-tenderloin)) (segfaults in ringbuffer code)
4. **Testing required**: Try TouchPad firmware with freedreno to see if it improves compatibility

**Recommendation:** Test with TouchPad firmware first, fall back to linux-firmware if issues occur

---

### 5. DSP (Qualcomm Hexagon QDSP6v2 - LPASS)
**Files (PIL - Peripheral Image Loader format):**
- `q6.mdt` (560 bytes) - Metadata file
- `q6.b00` (340 bytes) - Boot segment 0
- `q6.b01` (220 bytes) - Boot segment 1
- `q6.b02` (74 KB) - Code/data segment 2
- `q6.b03` (3 KB) - Code/data segment 3
- `q6.b04` (2.8 MB) - **Main firmware segment** (largest)
- `q6.b05` (739 KB) - Code/data segment 5
- `q6.b06` (673 KB) - Code/data segment 6
- `q6.b07` (48 KB) - Code/data segment 7

**Total Size:** ~4.3 MB
**Purpose:** LPASS (Low Power Audio Subsystem) audio/voice processing
**Load Method:** PIL (Peripheral Image Loader) via remoteproc subsystem
**Mainline Status:** ✅ Driver implemented (`drivers/remoteproc/qcom_q6v2_lpass.c`) ⭐
**Format:** Qualcomm PIL format (.mdt + .bXX segments)

**Driver Details:**
- Compatible: `qcom,msm8660-lpass-pil`, `qcom,apq8060-lpass-pil`
- Boot modes: PAS (TrustZone secure) and direct (untrusted)
- Uses MDT loader for firmware parsing
- LCC (LPASS Clock Controller) provides PLL4 clock
- Reserved memory at 0x8f000000 (5MB)

**Mainline Path:**
- `/lib/firmware/qcom/msm8660/q6.mdt` (and .bXX segments)

**Note:** Custom driver created for MSM8660/APQ8060 since the register layout differs
significantly from newer Hexagon versions (qcom_q6v5_* drivers).

---

### 6. VIDEO CODEC (VIDC 1.0)
**Files:**
- `vidc_1080p.fw` (489 KB)

**Purpose:** Video decoder/encoder firmware (1080p capable)
**Load Method:** Loaded by VIDC driver
**Mainline Status:** ❌ No MSM8660 VIDC support in mainline
**Capabilities:** H.264, MPEG-4, VC-1, VP8 encode/decode up to 1080p

**Note:** The mainline Venus driver only supports MSM8916 and newer SoCs.
MSM8660 uses VIDC 1.0 which is a different hardware block that would require
a separate driver implementation. Supported Venus SoCs:
- msm8916-venus, msm8996-venus, msm8998-venus, sdm660-venus, sdm845-venus, etc.

---

### 7. AUDIO CODEC (Wolfson WM8958)
**Files:**
- `wm8958_enh_eq.wfw` (2.4 KB) - Enhanced EQ algorithm
- `wm8958_mbc_vss.wfw` (5.3 KB) - Multiband Compressor + Virtual Surround Sound
- `wm8958_mbc.wfw` (4.2 KB) - Multiband Compressor only

**Purpose:** WM8958 DSP algorithm firmware  
**Load Method:** Loaded by wm8958-dsp driver via I2C  
**Mainline Status:** ✅ Supported by mainline wm8958 codec driver  
**Optional:** These are DSP enhancement algorithms, not required for basic audio

**Mainline Path:**
- `/lib/firmware/` (loaded by codec driver as needed)

---

## SUMMARY TABLE

| Component | Files | Total Size | Mainline Status | Priority |
|-----------|-------|------------|-----------------|----------|
| A6 Battery | 2 | 80 KB | ✅ Supported (ioctl) | Low (optional) |
| Touchscreen | 6 | ~450 KB | ✅ Supported | Low (optional upgrade) |
| WiFi (ath6k) | 8 | ~85 KB | ✅ Supported | **HIGH** |
| GPU (Adreno) | 4 | ~20 KB | ✅ Supported | Medium |
| DSP (QDSP6) | 9 | ~4.3 MB | ✅ Driver ready ⭐ | Medium |
| Video Codec | 1 | 489 KB | ❌ Not supported | Low |
| Audio DSP | 3 | ~12 KB | ✅ Supported | Low (optional) |

---

## CRITICAL FINDINGS

### 🚨 WiFi Board Data Files
**CRITICAL:** The WiFi firmware includes **separate board data calibration files** for WiFi-only and 3G models:
- `bdata.SD32.bin` - WiFi-only variant
- `3g_bdata.SD32.bin` - 3G variant

**These contain RF calibration data and are NOT interchangeable!**

**Action Required:**
1. Device tree must specify correct board data file based on hardware variant
2. Check if ath6kl driver can select bdata file from DT property

### 📋 Firmware Installation for Mainline

**Essential (for basic functionality):**
1. ✅ WiFi: Copy `ath6k/` directory to `/lib/firmware/`
2. ✅ GPU: Verify yamato firmware or use standard freedreno a200 firmware
3. ⚠️ Touchscreen: Optional - only needed for firmware updates

**Optional (for enhanced functionality):**
1. Audio DSP: WM8958 enhancement algorithms (✅ supported)
2. A6 firmware: For battery firmware updates via ioctl (✅ supported)

**Not supported in mainline:**
1. Video codec: VIDC 1.0 not supported (Venus is for newer SoCs)

**Now supported in mainline:** ⭐
1. Q6 DSP: LPASS PIL driver implemented (`qcom_q6v2_lpass.c`)

---

## FIRMWARE EXTRACTION COMMANDS

To copy essential firmware to a running system:

```bash
# WiFi firmware (ESSENTIAL)
sudo mkdir -p /lib/firmware/ath6k/AR6003/hw2.1.1/
sudo cp -r /home/herrie/webos/touchpad-kernel/doctor305/nova-cust-image-topaz.rootfs/lib/firmware/ath6k/hw2.1.1/* \
    /lib/firmware/ath6k/AR6003/hw2.1.1/

# GPU firmware (Adreno 220 uses Leia firmware, NOT Yamato)
sudo cp /home/herrie/webos/touchpad-kernel/doctor305/nova-cust-image-topaz.rootfs/lib/firmware/leia_*.fw \
    /lib/firmware/qcom/

# Audio DSP (optional)
sudo cp /home/herrie/webos/touchpad-kernel/doctor305/nova-cust-image-topaz.rootfs/lib/firmware/wm8958_*.wfw \
    /lib/firmware/

# Video codec (optional)
sudo cp /home/herrie/webos/touchpad-kernel/doctor305/nova-cust-image-topaz.rootfs/lib/firmware/vidc_1080p.fw \
    /lib/firmware/qcom/

# QDSP6 DSP (optional)
sudo mkdir -p /lib/firmware/qcom/msm8660/
sudo cp /home/herrie/webos/touchpad-kernel/doctor305/nova-cust-image-topaz.rootfs/lib/firmware/q6.* \
    /lib/firmware/qcom/msm8660/
```

---

## DEVICE TREE CONSIDERATIONS

### WiFi Board Data Selection

The device tree should specify which board data file to use:

**For WiFi-only variant (topaz.dts):**
```dts
&wifi {
    qcom,ath6kl-calibration-data = "bdata.SD32.bin";
};
```

**For 3G variant (topaz-3g.dts):**
```dts
&wifi {
    qcom,ath6kl-calibration-data = "3g_bdata.SD32.bin";
};
```

**Note:** Check ath6kl driver source to see if this property is supported, or if we need to use symlinks/file selection at runtime.

---

## NEXT STEPS

1. **Verify ath6kl board data selection mechanism** - Critical for WiFi functionality
2. **Test GPU firmware** - Check if yamato firmware works with freedreno
3. **Create firmware package** - For easy installation on target device

## MAINLINE SUPPORT STATUS

### Now Supported ⭐
1. ✅ **QDSP6 DSP (LPASS)** - Driver implemented: `drivers/remoteproc/qcom_q6v2_lpass.c`
   - Supports PAS (TrustZone) and direct boot modes
   - DT binding: `qcom,msm8660-lpass-pil` / `qcom,apq8060-lpass-pil`
   - Firmware path: `/lib/firmware/qcom/msm8660/q6.mdt`

### Not Planned (No mainline support)
1. ❌ **Video codec (VIDC 1.0)** - Would require new driver, Venus only supports MSM8916+

---

**Report Generated:** 2025-12-31 (Updated 2026-01-01)
**Source:** Legacy WebOS kernel doctor image
**Target:** Mainline Linux 6.x kernel
