# Update 9 for Gemini: corruption is GMEM-tile-aligned with 5 distinct masks

## TL;DR

* webOS = **70/70 same hash, no cycle**. Hardware works fine on KGSL.
* Mainline = period-8 cycle. The 7 "broken" outputs vary in *content* per
  boot but in a **structurally invariant pattern**.
* `0x0a56`, `0x0a57`, `0x0a58` (the undocumented MH per-port routing
  registers Gemini's analysis predicted) all read **bit-for-bit
  identical** between mainline and webOS. Bootloader sets them.
* The cycle is **GMEM-tile-aligned**. Each cycle position corrupts a
  specific bitmask of 12 GMEM tiles. The bitmask is stable across boots,
  the residue *content* in those tiles varies per boot.
* Only the triangle's **rendered fragments** are corrupted within
  affected tiles. Background (clear-color) pixels in the same tiles
  are intact. So the bug is at fragment output / wavefront stage, not
  GMEM resolve.

## The spatial breakthrough

100-cap test on 1024×768 framebuffer with 256×256 GMEM tiles (4 cols × 3
rows = 12 tiles). The rendered triangle covers tiles at:
* Row 0 (apex): col1, col2 (~24% coverage each)
* Row 1 (body): col1, col2 (~85% coverage), col0/col3 (~4% — edges only)
* Row 2 (base): col1, col2 (~85%), col0, col3 (~44%)
* Background: untouched

Per-cycle-position corruption mask (% of pixels in that tile that
differ from the cold-boot good `5adc3160` reference, threshold 5%):

```
GOOD (5adc3160) - 0% AE, all tiles 0% corrupted

low_31K (4% AE):
  row 0:   0   24   24    0
  row 1:   0    0    0    0
  row 2:   0    0    0    0
  -> only top row (apex tiles) bad

mid_149K (19% AE):
  row 0:   0   24   24    0
  row 1:   4   85   85    4
  row 2:   0    0    0    0
  -> top + middle rows bad

mid_169K (22% AE):
  row 0:   0    0    0    0
  row 1:   0    0    0    0
  row 2:  44   85   85   44
  -> only bottom row bad

hi_287K (36% AE):
  row 0:   0    0    0    0
  row 1:   4   85   85    4
  row 2:  44   85   85   44
  -> middle + bottom rows bad

highest_317K (40% AE):
  row 0:   0   24   24    0
  row 1:   4   85   85    4
  row 2:  44   85   85   44
  -> all three rows bad (max corruption)
```

Plus 2 more variants of "highest" with slightly different content.

**Key observations:**
1. Tiles with 85% corruption match where the triangle covers ~85-90% of
   the tile. Tiles with 4% match the triangle just barely clipping a
   corner. So **only triangle fragments are corrupted; the cleared
   background is intact**.
2. The same set of "bucket sizes" (31K, 149K, 169K, 287K, 317K, 318K
   ×2) appears in **all 3 mainline test runs** (pre-IN_FLIGHT_LIMIT,
   post-IN_FLIGHT_LIMIT, current MH-probe build) with identical
   counts — even though the actual hashes differ across boots. The
   cycle's *structure* is fixed; only the residue content varies.

## What this means

The bug isn't:
* MH per-port routing (matches webOS bit-for-bit — confirmed)
* MH ARBITER IN_FLIGHT_LIMIT (we cleared bit 15, didn't fix the cycle)
* Memory residue in GMEM as a whole (background pixels are intact)
* GMEM tile resolve / gmem2mem (resolve writes the whole tile —
  including background — and background is correct, so resolve worked)

The bug appears to be:
* The **fragment/wavefront output for the triangle** is wrong on
  specific tiles in specific cycle positions
* It's per-tile because GMEM rendering is per-tile (each tile is its
  own RB context)
* It's per-cycle-position because *something* is rotating per fresh
  DRM context

## Hypothesis: SQ wavefront slot assignment cycles

The SQ has internal wavefront slots. We've ruled out per-`CP_DRAW_INDX`
slot advance (the 8x and 5x clear-amplification tests showed zero
shift). But **per-tile slot assignment** could still be the source:

* GMEM rendering processes 12 tiles per batch
* Each tile launches some number of wavefronts (depending on triangle
  coverage in that tile)
* If wavefront slots are assigned round-robin across tiles, with N
  slots and 12 tiles, the pattern is `12 mod N` per batch
* Across multiple batches in one process, the slot pointer advances by
  N per batch
* Across processes, batch counter increments → slot offset per process
  shifts by 1 — giving the period-8 cycle

12 tiles × ? wavefronts per tile, mod 8 slots... we don't have a clean
model yet. But the per-tile bitmask of "which tiles end up on bad
slots" matches what we see.

The "good" cycle position (`5adc3160`) is when ALL tiles happen to
land on whichever subset of slots that have correctly-initialized
parameter SRAM.

## The R↔B byteswap signature still holds

The corrupted fragments show a near-perfect R↔B swap with G untouched
(channel-mean delta R≈34, G≈6, B≈34 on the worst pre-fix sample). This
is the LE↔BE byteswap fingerprint at the RB stage — meaning whatever's
wrong with those wavefronts is producing fragment data with reversed
component byte order.

If 7 of 8 SQ slots have stale `MH_CLNT_INTF_CTRL_CONFIG2`-equivalent
bits cached internally (uncleared per-slot byteswap state), the
fragments emitted from those slots would output BGR instead of RGB.

## Asks

1. **A22X per-tile wavefront slot accounting**: do you know how many
   wavefronts per tile the SQ launches for a single triangle covering
   ~85% of a 256x256 tile? If we can compute `12 tiles × W
   wavefronts mod 8 slots`, we can confirm the model.

2. **Per-slot endianness/byteswap state**: is there a per-SQ-slot
   register or shadow state that holds endianness/byteswap config,
   distinct from the per-port `MH_CLNT_INTF_CTRL_CONFIG1/2` we already
   matched? KGSL's hw_init must be initializing this somehow even
   though we don't see explicit writes.

3. **The "SQ instruction prefetch / parameter SRAM" theory**: Earlier
   you suggested 7 slots have toxic VFETCH descriptor / GPR state.
   Is there a way to **forcibly reset all 8 slots** at hw_init? The
   `CP_LOAD_CONSTANT_CONTEXT` packet is what KGSL uses for full state
   broadcast across slots, but freedreno avoids it on a2xx because
   the hardware shadow-memory mechanism hangs the GPU.

4. **The boot-dependent residue content**: the corruption *content*
   varies per boot (different broken hashes each test) but the
   *structure* is invariant (same tile bitmasks, same AE buckets).
   This means whatever is "bad" in slots 1-7 is **boot-dependent
   uninitialized memory** (parameter SRAM never cleared). What's in
   parameter SRAM at cold boot vs after LSM has run? Is there an
   explicit way to clear it?

## Next experiments we're considering

* **`memset(0)` on GPUMMU page table at probe**: clears any leaked
  residue at IOVA mappings.
* **VA_RANGE alignment to webOS** (`GPUMMU_VA_START = 0x66000000`):
  the only other observed mainline-vs-webOS register difference.
* **Per-submit MH state dump**: see if anything cycles between
  consecutive submits within one hw_init session (we only sampled at
  hw_init time so far; per-submit cadence is finer).
* **Pin LSM compositor** off / kill `surface-manager` before running
  the test — see if cycle persists or disappears (would tell us if the
  residue source is LSM-frame-dependent or kernel/firmware-only).

What's your top recommendation given the tile-aligned corruption
breakthrough?
