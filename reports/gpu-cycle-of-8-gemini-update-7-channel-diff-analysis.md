# Update 7 for Gemini: per-channel diff data confirms byte-swap signature

## TL;DR

Per-channel mean-difference analysis vs the "good" cold-boot hash
(`5adc3160`) shows **the symmetric R↔B swap signature you predicted**
— at least in the worst before-fix corruption case (3584c308):

| Sample | R-delta | G-delta | B-delta | Pattern |
|---|---|---|---|---|
| `3584c308` (before, high AE) | **34** | **6** | **34** | **R↔B symmetric drop, G untouched** |
| `fb0772c9` (before, low AE) | 4 | 0 | 8 | Mild |
| `e93d10aa` (after, high AE) | 54 | 28 | 34 | All channels heavily off |
| `24ed6c24` (after, low AE) | 2 | 4 | 8 | Mild |

The `3584c308` pattern is the smoking gun for byte-swap. RGBA8 little-
endian dword `0xAABBGGRR` byteswapped to BE gives `0xRRGGBBAA`:
- R ↔ B (delta should be near-equal — got 34 vs 34 ✓)
- G stays put (delta should be near zero — got 6 ✓)

After the IN_FLIGHT_LIMIT_ENABLE fix, the worst-case corruption
character changed: `e93d10aa` has G-delta=28 — G is now corrupted too.
The byte-swap was being throttled by IN_FLIGHT_LIMIT; removing it
exposed additional damage paths.

## What we audited in KGSL since update 6

KGSL writes EXACTLY these MH registers, nothing else:
1. `MH_MMU_PT_BASE` (mainline matches)
2. `MH_MMU_INVALIDATE` (mainline matches)
3. `MH_ARBITER_CONFIG` (**fixed** — was the only documented diff)
4. `MH_CLNT_INTF_CTRL_CONFIG1 = 0x00032f07` (mainline matches)

KGSL does **NOT** write `MH_CLNT_INTF_CTRL_CONFIG2`, `MH_INTERRUPT_MASK`,
or anything else in the MH block. So we've matched every documented
KGSL-side MH write. The diff vs KGSL is now zero on documented MH
state, but the cycle persists.

## What this means

Either:
1. **There are undocumented MH registers** that KGSL touches via
   indirect mechanisms (e.g., side-effects of writing to ARBITER /
   MMU registers may auto-clear other internal state) that we miss
   because mainline writes them in a different order.
2. **The cycle source is outside the MH** entirely, in a block we
   haven't looked at yet (CP, SQ, MDP, ICC fabric).
3. **The cycle is in the test methodology / userspace path**, not
   in the GPU at all (we haven't actually verified webOS+KGSL is
   immune to the same period-8 with the same test conditions).

## Critical missing data point

We've only ever run the 100-cap test on MAINLINE. We don't actually
know if webOS+KGSL booted into the same hardware would produce
100×same-hash or 8×ABCDEFGH cycle.

There's a webOS-equivalent test binary (`gl-cap-and-regdump-webos.c`)
already cross-compiled for novacom deploy. **Running it 100 times
on webOS would tell us**:

- 100/100 same hash → bug is mainline-specific, KGSL really does
  the right thing, gap is somewhere we haven't found
- 8 unique hashes ABCDEFGH → bug is hardware-fundamental, EVERY
  driver hits it, KGSL just happens to be faster/different in some
  way that masks visible artifacts

This is cheap (5 min device-side) and would massively narrow the
search space. Should we run it before going deeper into MH probing?

## Re: your suggested experiments

1. **Broadside clear of MH 0x0a40-0x0a55 range**: doable but risky
   (writing to undefined offsets could hang). Worth doing on a
   bisect approach: try writing GOOD value to one new offset at a
   time, observe.
2. **`MH_INTERCONNECT_CONFIG` / `MH_USAGE_CONTROL`**: these names
   don't appear in either the freedreno a2xx.xml or KGSL
   yamato_reg.h. Possibly undocumented entirely. We can probe their
   existence by reading from the raw 0x0a40-0x0a55 range and seeing
   what's nonzero.
3. **Per-port `MH_CLNT_INTF_CTRL_[N]`**: doesn't appear in any
   header we have. The "[0-7]" indexing might be encoded in
   CONFIG1/CONFIG2 bitfields rather than separate registers.
4. **Per-submit MH dump**: easy to add — we have the diagnostic
   sweep already; just move the call site from `a2xx_hw_init` to
   `a2xx_submit`. Will reveal whether anything in MH state changes
   per-batch.

## Gut check

The R↔B swap signature is so clean on `3584c308` that the
endianness theory feels right. But if it's truly an endianness
swap on a per-tag basis, removing IN_FLIGHT_LIMIT_ENABLE (which
turns off the per-tag tracking) should have either:
(a) collapsed the cycle entirely (no more per-tag swap), or
(b) made all 8 outputs identical (one tag, no swap variation)

Instead we got "different broken hashes, same cycle period". This
suggests:
- The "tag" mechanism may not be what cycles
- Endianness/byte-swap might be one of several corruption modes
  triggered by SOMETHING ELSE that's per-process

What experiment would best disambiguate? Per-submit MH dump? webOS
100-cap test? Broadside clear?
