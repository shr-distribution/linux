# CE2 chunked-AES round 2 brief — accumulating failure past N chunks

## Where we are

Implemented your previous recommendation (software-driven chunking
with FIRST/MIDDLE/LAST SEG_CFG management per 64 B AES chunk).  The
fundamental approach works — we have lifted the silicon 4-block
(64 B) cap.  But there's a residual issue that scales with chunk
count.

## What's working

- 12/12 NIST vectors pass (AES-128 ECB/CBC/CTR, AES-256-CBC, DES-CBC,
  3DES-CBC, encrypt + decrypt each).  3DES-CBC decrypt is
  intermittently flaky but that pre-dates the chunking work.
- AES-CBC `qce-streamenc` vs `openssl enc -nopad`:
  - 64 B (1 chunk): 100% match
  - 128 B (2 chunks): 100% match
  - 256 B (4 chunks): 3/3 runs match
  - 512 B (8 chunks): 2/3 runs match
  - 1 KB (16 chunks): 1-2/3 runs match
  - 2 KB (32 chunks): 2/3 runs match
  - **4 KB (64 chunks) and larger: 0/3 runs match — always fails**

## Per-chunk failure pattern

Failure rate is roughly ~3-7% per chunk and clearly cumulative.  Past
~64 chunks the operation deterministically fails at a random byte
offset.  Two consecutive 4 KB runs with identical plaintext diverge
at different bytes (688, 2544, 3568, 496 across 4 runs).

When the chained chunks fail mid-stream, the engine's STATUS register
typically shows DOUT_ERR=1 + SW_ERR=1 + ENCR_BUSY=1 + DIN_RDY=1 with
CRYPTO_STATE=PROCESSING (4) — engine is stuck mid-block claiming
it wants more input.

## What we DO between chunks

For every chunk past the first:

1. **Reset cycle** (currently testing the deepest variant):
   - `clk_disable(qce->core)` → 10 us → `clk_enable(qce->core)`
   - SW_RST pulse: writel(BIT(0)) to CONFIG, 10 us, writel(0), 10 us
   - Poll STATUS until CRYPTO_STATE == IDLE (state 0)
2. **Re-write engine config**:
   - `writel(0, AUTH_SEG_CFG)`
   - Re-expand AES round-key schedule and `writel` all 60 words to
     `AES_RNDKEY0..59` (SW_RST clears the key bank on this silicon —
     empirically verified)
   - For DES/3DES, writel `DES_KEY0..5`
   - `writel(0xffff, CNTR_MASK)` for CTR mode
3. **Re-install IV** (CBC/CTR chaining):
   - For each chunk past the first, read `CNTR0..3` after the prior
     chunk's DMA completes (this gives the last cipher block) and
     write it back to `CNTR0..3` before this chunk's GOPROC.
   - Confirmed empirically that:
     - CNTR readback gives the correct last cipher block (matches
       openssl's nth ciphertext block)
     - The writeback sticks (read pre-GO shows the value we wrote)
     - The engine reloads from CNTR on SEG_CFG.FIRST=1 (chunk 1's
       output for the FIRST 64 B is now correct in all 0-byte-zero
       plaintext tests)
4. **Standard SEG_CFG sequence** per chunk:
   - SEG_CFG with FIRST | LAST | ENCR_ALG=AES | key-size bits | ENCODE | AUTH_POS
   - ENCR_SEG_CFG = chunk_len << 16
   - SEG_SIZE = chunk_len
   - CONFIG (interrupt masks + high-speed enables)
   - GOPROC
   - poll STATUS until CRYPTO_STATE != IDLE
5. **DMA** (qcom_adm BOX descriptor, non-FC mode — flow-control
   experiments hit different deadlocks).  Burst = 64 dw (AES) or
   8 dw (DES).
6. **Capture CNTR0..3 after DMA completion + wait for engine to
   return to IDLE** for next iteration.

What's "fresh" at the start of each chunk by our model:
- Engine state machine: reset to IDLE via SW_RST
- Key bank: cleared by SW_RST, then re-written
- IV: rewritten from prior chunk's CNTR readback
- AUTH state: AUTH_SEG_CFG cleared, no auth alg in SEG_CFG
- DIN/DOUT FIFOs: presumably cleared by SW_RST
- Clock domain: cycled via clk_disable/enable (this commit)

## The mystery

Something accumulates across chunks that NONE of:
- `reset_control_assert/deassert` (full GCC_CE2_RESET)
- `clk_disable / clk_enable` on the core clock
- SW_RST pulse via CONFIG[0]
- Key bank re-write
- DMA channel `dmaengine_terminate_sync` after each transfer

...clears.  The engine works correctly for the first 30-60 chunks
and then progressively breaks.

## Possible candidates we haven't directly addressed

1. **DOUT FIFO drain**: STATUS shows DOUT_SIZE_AVAIL in dwords.  After
   our DMA tx-completion, is DOUT_SIZE_AVAIL guaranteed to be 0, or
   might there be dwords sitting that the ADM didn't drain?  If
   so, what's the right way to drain DATA_OUT explicitly?
   The CE2 register map shows `DATA_OUT` at offset 0x010 separately
   from `DATA_SHADOW0` at 0x8000.

2. **Engine internal op counter / sequence number** that wraps or
   saturates after N ops.  Some Qualcomm engines have an EE/SD/CI
   architecture where the controller maintains per-channel sequence
   numbers; ADM in particular has `CH_RSLT` results that the
   controller accumulates.

3. **CRCI handshake state on ADM**: even though we use BOX (non-FC)
   mode, the CRCI lines from CE2 to ADM may be implicitly handshaking
   per-burst and accumulating asserted-without-being-cleared state.

4. **Shared peripheral interconnect / ICC fabric BW vote**:
   crypto uses interconnect path 'crypto-mem' (or similar).  Repeated
   ops without renewing the BW vote might starve the engine after a
   while.

5. **Cache flush / dma_sync state**: our bounce buffers are
   `dma_alloc_coherent` so cache-coherent.  But we ALSO do
   `sg_pcopy_to_buffer` + `sg_pcopy_from_buffer` per chunk, which
   touches CPU caches.  Maybe a stale cache line for one of the
   intermediate buffers gets DMAd by chunk N+1 occasionally.

6. **Engine clock rate**: do we need `clk_set_rate(qce->core, N)`
   for sustained multi-chunk throughput?  Currently we accept whatever
   the bootloader left it at.

## Concrete asks

1. **What state survives SW_RST + qce->core clock cycle on CE2?**
   The Qualcomm "Crypto Engine Architecture" doc (which I don't
   have a copy of) presumably enumerates this.  Specifically:
   does ANY internal register / latch / counter persist across
   both SW_RST and core-clock-disable?  If so, what's the canonical
   way to clear it?

2. **Is there a DOUT FIFO drain sequence we're missing?**  e.g.,
   "after each op, read DATA_OUT until DOUT_SIZE_AVAIL == 0".  Or a
   specific register write to flush the FIFO.

3. **The deterministic-failure-past-64-chunks pattern**: have you
   seen this in other Qualcomm CE work, and does it have a known
   cause?  Op-counter wrap?  Internal queue saturation?

4. **`reset_control_assert(GCC_CE2_RESET)` vs `clk_disable(qce->core)`
   — what's the actual scope of each on this silicon?**  Are they
   equivalent, complementary, or one a superset of the other?

5. **Does CE2 require explicit `CE2_REG_REGISTER_LOCK` (offset 0x02C)
   management for multi-op streams?**  The register exists but we
   never touch it.  Some Qualcomm engines use a per-op lock that's
   acquired before op and released after — if we're not releasing,
   the engine might block subsequent ops silently.

## Files / artifacts

- Current driver source: `drivers/crypto/qce/common.c`, function
  `qce_ce2_pio_run_skcipher`, in branch
  `tenderloin/6.18/upstream-patches` (HEAD ~ e6981bb4f447 at time
  of writing).
- Test tool: `/tmp/qce-test/qce-streamenc.c` (AF_ALG file streamer).
- Register map: `drivers/crypto/qce/regs-ce2.h`.
- Prior brief (FIFO depth + chunking proposal): `gemini-brief.md`.
