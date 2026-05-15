---
created: "2026-04-14"
last_edited: "2026-04-14"
---
# Implementation Tracking: Core

Build site: context/plans/build-site.md

| Task | Status | Notes |
|------|--------|-------|
| T-004 | DONE | Verified: PA 1 pad L1890-1891, IFP 2 pads L1917-1919, link L1470-1473, async L1986, cleanup L2022-2029 |
| T-006 | DONE | Verified + fixed: 5 formats (UYVY,YUYV,RGB565,SGRBG8,SGRBG10), enum L1315-1333, PA enum L1206-1214 |
| T-009 | DONE | Verified: read 0x0000 L1770, check 0x2480 L1776, fail L1780, after power L1866 |
| T-010 | DONE | Verified: write L256-257, read L265-268, poll L275-293 |
| T-012 | DONE | Implemented: enum_frame_interval L1443-1468, g_frame_interval L1470-1488 |
| T-014 | DONE | Verified: enum_frame_size L1407-1441, set_fmt clamping L1510-1516 |

## Verification Details

### T-004: Dual Subdevice Registration
- ✓ PA 1 source pad: L1890 `sensor->pa.pad.flags = MEDIA_PAD_FL_SOURCE`, L1891 `media_entity_pads_init(..., 1, ...)`
- ✓ IFP sink+source: L1917-1918 `pads[0]=SINK, pads[1]=SOURCE`, L1919 `media_entity_pads_init(..., 2, ...)`
- ✓ Link PA→IFP: L1470-1473 `media_create_pad_link(&sensor->pa.sd.entity, 0, &sensor->ifp.sd.entity, 0, ...)`
- ✓ V4L2 async: L1986 `v4l2_async_register_subdev(&sensor->ifp.sd)`, PA registered in callback L1463
- ✓ Clean unregister: L2022-2029 cleanup calls, L1451-1456 `mt9m113_ifp_unregistered()`
- ✓ Separate ctrl handlers: L1896 `v4l2_ctrl_handler_init(&sensor->pa.hdl, ...)`, L1924 `v4l2_ctrl_handler_init(&sensor->ifp.hdl, ...)`

### T-006: Format Enumeration
- ✓ IFP enumerates UYVY8_1X16: L211 in format_infos array, L1328-1331 enum
- ✓ IFP enumerates YUYV8_1X16: L215 in format_infos array
- ✓ IFP enumerates RGB565_1X16 (FIXED): Added L218-221 in format_infos array
- ✓ PA enumerates SGRBG10_1X10: L1206-1214 `mt9m113_pa_enum_mbus_code`
- ✓ Invalid index returns -EINVAL: L1210-1211 (PA), L1321-1322, L1328-1329 (IFP)
- ✓ get_fmt: L1261 (PA), L1491 (IFP) use `v4l2_subdev_get_fmt`
- ✓ set_fmt validates: L1492 `mt9m113_ifp_set_fmt`
