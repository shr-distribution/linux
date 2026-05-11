# Fork C v2 — 8-hash cycle analysis (2026-05-11)

## Source

Mesa patch 0093 v2 (`OUT_PKT0` fix for sub-0x2000 register writes) applied:
- libgallium md5: `4d9f9da827528c203d7d50c190c63a73`
- kernel: `6.18.0-luneos-g4a4a6622f1ae`

Captures pulled live from device after fresh sysrq reboot. **All 8 bin files
are byte-identical** to `reports/fb-captures/perturbed-7-wrong-hashes/`
(verified via `cmp`) — Fork C v2 reproduces an earlier "perturbed" cycle
seen at 13:35 today, before Fork C work began. This means Fork C v2's added
VSC config + 0xC00=1 + LRZ_VSC_CONTROL=3 produces output indistinguishable
from whatever was on device at 13:35 — the cycle is unchanged from that
perturbed state.

## The 8 cycle hashes

```
8cbc2640 → d71843ae → 2c65153c → 15ad14f0 → add5096d → 27b74302 → 6ed2ef94 → 5adc3160 → (repeat)
```

`5adc3160` is the **bit-exact-correct** render (1 of 8 = 12.5% correct).

## Failure mode: per-vertex visibility cycling, NOT whole-tile cover

Earlier analysis suspected "tile coverage failures" (whole 341x384 GMEM
tiles rendered or skipped). Per-tile nonzero-pixel counts however are
identical across all 8 hashes — every hash touches every tile.

The actual failure is **per-vertex contribution loss**, exposed by per-tile
mean RGB delta vs the correct render:

```
Reference (5adc3160 CORRECT): per-tile mean RGB
  T00=(  2,  0,  2)  T01=( 23, 23, 97)  T02=(  0,  2,  2)
  T10=( 81, 12, 22)  T11=( 85, 85, 56)  T12=( 12, 82, 22)

Wrong hashes: delta R,G,B from reference (- = lost intensity)
hash       | T00              T01              T02              T10              T11              T12
========== | ==============   ==============   ==============   ==============   ==============   ==============
15ad14f0   |   -2/  +4/  -1   -18/ +22/ -68    +0/  -2/  -2    -80/ +11/  -7   -79/ -48/ -30   -12/ -82/ -22
27b74302   |   -2/  +4/  -1   -20/ +16/ -83    +0/  -2/  -2    -12/ +19/  +3   -18/ +10/ -13    -1/ -14/ -10
2c65153c   |   +0/  +0/  +0    -3/ +18/ -43    +0/  +0/  +0     +0/  +0/  +0    +0/  +0/  +0    +0/  +0/  +0
6ed2ef94   |   -2/  +1/  +0   -21/  -5/ -77    +0/  -2/  -2    -81/  -7/ -10   -52/ -18/ +14   +43/  -8/ +42
8cbc2640   |   -2/  +0/  -2   -20/  +7/ -89    +0/  -2/  -2    -81/ -12/ -22   -56/ -23/  -3   +43/  -5/ +45
add5096d   |   +0/  +0/  +0    +0/  +0/  +0    +0/  +0/  +0    -68/  -8/ -10   -60/ -58/ -16   -11/ -69/ -12
d71843ae   |   -2/  +0/  -2   -17/ -11/ -46    +0/  -2/  -2    -81/ -12/ -22   -68/ -76/ -18   +19/ -74/ +24
```

Reference T01 is the blue vertex region, T10 is red, T12 is green. The
deltas show specific colour channels going missing per hash:

| Hash | Lost vertices | Notes |
|------|---|---|
| 5adc3160 | NONE (correct) | golden render, present at cycle position 8/8 |
| 2c65153c | partial B | only T01 dB=-43; 5/6 tiles correct — **closest to right** |
| 27b74302 | B (mostly) | T01 dB=-83; minor R/G drift |
| add5096d | R+G in bottom row | top row correct, bottom row deltas large |
| d71843ae | R + part G | T10 dR=-81, T12 dG=-74 |
| 6ed2ef94 | R + partial B | T10 dR=-81, T01 dB=-77, +43 residue in T12 |
| 8cbc2640 | R + B | T10 dR=-81, T01 dB=-89, +43 residue in T12 |
| 15ad14f0 | R + G + part B | worst case — multiple vertices lost |

## Comparison vs baseline (pre-Fork-C-anything)

| Property | Baseline (`option-d-mask-0x1F/phase-B-no-pulse`) | Fork C v2 / perturbed-7 |
|---|---|---|
| Failure mode | Wrong-colour cycling (same triangle shape, varying RGB intensities) | Per-vertex contribution cycling |
| 8 hashes | 070bdc57 259d419d 48845819 5adc3160 73bb37bb 9e25589e acb14db9 ccb21b89 | 15ad14f0 27b74302 2c65153c 5adc3160 6ed2ef94 8cbc2640 add5096d d71843ae |
| Correct in cycle | 5adc3160 (1 of 8) | 5adc3160 (1 of 8) |
| `5adc3160` bin md5 | identical | identical |
| Position of correct | varies | varies |

The `5adc3160` capture is bit-exact-identical between baseline and Fork C v2
— when rendering happens to come out right, the output matches. The wrong
hashes differ entirely between the two sets, reflecting different binner
state machine modes.

## Conclusion

Fork C v2 successfully engaged the A22X hardware binner (no hangs, 30/30
captures completed) but **the period-8 cycle persists with a different
hash set than baseline**. The binner is doing per-vertex visibility
decisions in 8 stable but wrong modes.

Most informative single observation: **2c65153c** has only 1 tile wrong
(T01 with partial blue loss). It is the closest a non-correct hash gets
to correctness. The 8 modes are not random — they form a stable
deterministic cycle of progressively worse decisions about which vertex
contributions to ship to the rasterizer.

## Next experiment candidates

1. **Add SQ_GPR_MANAGEMENT = 0x7f010** — the only piece of the webOS
   prelude (`leia_configure_binning_pass`) we deliberately skipped. Hypothesis:
   sets the binning shader register partition that the binner needs in
   order to make consistent per-vertex visibility decisions.
2. **Implement a real binning pass IB** — emit a stripped-down VS-only
   draw stream during binning mode, then disengage and emit the regular
   render pass. This is the full A22X two-pass model webOS uses.
3. **Investigate cycle phase** — capture the per-cycle-position register
   set (CP_SCRATCH, VSC_PIPE state) and check whether anything visibly
   varies across the 8 hashes. Earlier "register invariant" finding was
   pre-Fork-C-v2; needs re-verification.
