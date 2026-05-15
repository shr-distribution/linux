---
created: "2026-04-14"
last_edited: "2026-04-14"
---
# Implementation Tracking: Controls

Build site: context/plans/build-site.md

| Task | Status | Notes |
|------|--------|-------|
| T-016 | DONE | Verified: SEQ_CMD writes L1154-1160, poll L1163, 500ms timeout |
| T-017 | DONE | Implemented: HFLIP control L1683-1715, read_mode bit 0 |
| T-018 | DONE | Implemented: VFLIP control L1717-1749, read_mode bit 1 |
| T-019 | DONE | Implemented: menu control L2797, handler L2361, disables MCU for test patterns |
| T-020 | DONE | Implemented: EXPOSURE_AUTO L1847-1854, AE_GATE enable/disable |
| T-021 | DONE | Implemented: ANALOGUE_GAIN L1861-1864, register 0x3028 |
| T-022 | DONE | Implemented: AUTO_WHITE_BALANCE L1866-1869, awb_mode |
| T-023 | DONE | Verified: COLORFX control L1751-1793, effects 0x2759/0x275B |
| T-024 | DONE | Implemented: POWER_LINE_FREQUENCY L1795-1822, fd_mode 0xA404 |
| T-025 | DONE | Implemented: SATURATION L1824-1832, awb_saturation 0xA354 |
| T-027 | DONE | Implemented: EXPOSURE L1856-1859, coarse_it_time 0x3012 |

## Verification Details

### T-016: Context Switching
- ✓ SEQ_CMD_RUN (0x01) for Context A: L1157-1158
- ✓ SEQ_CMD_CAPTURE (0x02) for Context B: L1154-1155
- ✓ SEQ_CMD_REFRESH (0x05): L942-943, L1701-1702
- ✓ Polling after seq_cmd: L935, L947, L1163, L1703
- ✓ 500ms timeout: poll with 500ms param

### T-017: Horizontal Flip Control
- ✓ V4L2_CID_HFLIP registered: L2135
- ✓ Sets bit 0 of read_mode_a (0x2717): L1698-1699
- ✓ Sets bit 0 of read_mode_b (0x272D): L1706-1707
- ✓ Issues REFRESH command: L1710-1713

### T-018: Vertical Flip Control
- ✓ V4L2_CID_VFLIP registered: L2137
- ✓ Sets bit 1 of read_mode_a (0x2717): L1730-1731
- ✓ Sets bit 1 of read_mode_b (0x272D): L1738-1739
- ✓ Issues REFRESH command: L1742-1745

### T-023: Color Effect Control
- ✓ V4L2_CID_COLORFX menu registered: L2139-2141
- ✓ NONE=0x6440, BW=0x6441, SEPIA=0x6442, NEG=0x6443, SOLAR=0x6445
- ✓ Writes to both 0x2759 and 0x275B: L1781-1783
- ✓ Issues REFRESH command: L1787-1790

### T-024: Power Line Frequency Control
- ✓ V4L2_CID_POWER_LINE_FREQUENCY menu registered: L2143-2146
- ✓ Supports Disabled, 50Hz, 60Hz, Auto
- ✓ Writes to fd_mode (0xA404): L1821

### T-025: Saturation Control
- ✓ V4L2_CID_SATURATION registered: L2147-2148
- ✓ Range 0-255, default 128
- ✓ Writes to awb_saturation (0xA354): L1826-1827
- ✓ Issues REFRESH command: L1828-1831
