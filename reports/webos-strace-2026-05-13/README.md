# webOS strace breakthrough — 16 IBs per render = cycle source

**Date:** 2026-05-13
**Device:** booted to stock webOS (CyanogenMod-era / Adreno KGSL kernel)
**Binary traced:** `tools/gpu-regdump/gl-cap-and-regdump-webos` (same hello-triangle as mainline cap, ported to webOS PalmPDK)

## TL;DR

`strace -e trace=ioctl` of one webOS triangle render shows **16 distinct `KGSL_IOCTL_RINGBUFFER_ISSUEIBCMDS` (0xc0140910) submits per render** on `/dev/kgsl-3d0` (fd 19).  Mainline Mesa freedreno submits at most 2 IBs per render (binning IB + per-tile loop IB via Fork A/B).  This perfectly explains the period-16 cycle:

- The GPU's internal state-cycle counter advances **once per IB submission**, mod 16.
- webOS: 16 advances/render → counter returns to same phase every render → no visible cycle.
- Mainline: 2 advances/render → counter walks through cycle one frame at a time → visible period-16 cycle of complete frames.

## Counts (one render, KGSL ioctls on fd 19)

```
   16 ioctl(19, 0xc0140910, ...)   IOCTL_KGSL_RINGBUFFER_ISSUEIBCMDS  ← THE SUBMIT
   17 ioctl(19, 0xc00c0923, ...)   IOCTL_KGSL_DEVICE_WAITTIMESTAMP_CTXTID
    4 ioctl(19, 0xc00c0902, ...)   IOCTL_KGSL_DEVICE_GETPROPERTY
    4 ioctl(19, 0x40080906, ...)   IOCTL_KGSL_DEVICE_REGREAD
    1 ioctl(19, 0xc0080913, ...)   (DRAWCTXT_CREATE or similar)
   17 ioctl(19, 0x40040921, ...)   IOCTL_KGSL_CMDSTREAM_FREEMEMONTIMESTAMP
    3 ioctl(19, 0xc0100920, ...)   IOCTL_KGSL_SHAREDMEM_FREE
```

## Submit-window structure (between first and last ISSUEIBCMDS)

```
13× ISSUEIBCMDS at addr 0x7e9db95c  (same stack struct, reused per call)
 1× REGREAD       at addr 0x7e9db930
 2× ISSUEIBCMDS at 0x7e9db83c, 0x7e9db88c   (different stack frames)
 1× REGREAD       at 0x7e9db860
 1× WAIT_TIMESTAMP at 0x7e9db9f4
 2× ISSUEIBCMDS at 0x7e9db89c, 0x7e9dbb04   (final two)
```

The "13 fast IBs in a tight loop" pattern matches a per-tile / per-pipe submission scheme — likely binning-pass setup + 12 tile-related IBs. Then resolve / cleanup with the REGREAD-WAIT-IB pattern.

## Visual confirmation

`webos-render.png` shows the **textbook clean RGB triangle** — blue top, red BL, green BR, with the dark blue (`0.10 0.20 0.30 1.0`) clear color visible.  No tile mosaic, no vertex-color rotation.  This is what mainline only produces 1 in 16 caps as `5adc3160`.

(Note: webOS framebuffer dimensions / colorspace differ slightly from mainline cap output, so the hash `abda39bc2c6c69367a65f2dc685ff20e` doesn't match `5adc3160` directly — but visually they're the same correct triangle.)

## Implications for the Mesa fix

The GPU has some internal state (most likely a **per-IB SQ wavefront-slot allocator pointer** or **binner state-machine index** counted mod 16) that:
- Increments by 1 every `IOCTL_KGSL_RINGBUFFER_ISSUEIBCMDS`
- Determines which constant-residue snapshot the wavefront slots use for the render
- Only 1 of 16 phases happens to hold the correct user state for the current draw

**Fix options to test:**

1. **Submit (16 − N_real) NOP IBs per render** to align the counter to phase 0 before each real render.  Lightest possible:  a small NOP IB (1 dword `CP_NOP`) submitted 14 times before the real Fork A/B IBs.

2. **Investigate if the counter is per-IB inside one ioctl** (i.e., per `CP_INDIRECT_BUFFER` call), not per ioctl.  If yes, even cheaper: emit 14 extra `CP_INDIRECT_BUFFER` calls to a 1-dword NOP IB within the existing Mesa cmdstream.

3. **Replicate webOS's per-tile submit cadence** — break the per-tile loop into per-tile separate ioctls so each tile = 1 submit.  Closer to webOS architecturally but much heavier driver change.

Option 2 is the cheapest test; if the counter is per-IB-inside-ioctl this single Mesa patch fixes the cycle.

## Open data still needed

- The actual `ibaddr` and `sizedwords` of each of the 16 IBs (to see what's in them).  strace shows the `kgsl_ringbuffer_issueibcmds` struct pointer but not its contents; would need either `strace -v` with verbose-struct decoding or a one-off ptrace tool.
- Whether the binning IB content per tile is the same or different (the 13 fast IBs at the same stack addr suggest a tight loop emitting similar IBs).

## Files

| file | content |
|---|---|
| `webos-cap.bin`    | webOS framebuffer dump (1024×768 RGBA8, surface format may differ from mainline) |
| `webos-render.png` | rendered clean triangle |
| `webos-regs.txt`   | post-render register dump via KGSL ioctl path |
| `strace-gl-cap-webos.log` | full strace of `gl-cap-and-regdump-webos` (~66k ioctls; 16 KGSL submits identifiable by `0xc0140910`) |
