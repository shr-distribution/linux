# Update 13 for Gemini: cycle is back, RBBM poll is decorative, wptr_delay broke, WPTR-polling mode hangs at ME_INIT

## TL;DR

Three things since update 12:

1. **The 8-cycle is back and reproducing cleanly.** 100/100 runs from
   a fresh boot, *zero* MMU faults, *zero* hangchecks, perfect
   `ABCDEFGH ABCDEFGH ...` pattern. `5adc3160` (the correct render)
   sits at exactly 1/8 = 13/100. Update 12's "cycle vanished"
   conclusion was wrong, or at least transient.

2. **The targeted RBBM_STATUS::GUI_ACTIVE poll is decorative.** With
   `a2xx_rbbm_poll_enable=Y` (default) we get the 8-cycle. With
   `=N` we get the same 8-cycle, same hashes, same distribution.
   The GUI_ACTIVE bit clears too early to catch whatever the
   `udelay(10000)` was masking.

3. **`wptr_delay=10000` no longer collapses the cycle either.** Same
   8 unique hashes, same 12-13 distribution. This is a regression
   vs update 11 where 10 ms reliably gave 100/100. Either the
   rebuilt kernel/userspace is materially different in some way I
   haven't identified, or we never had a clean A/B in the first
   place. We have the artifacts saved (8 sample bins per condition)
   for forensic comparison if useful.

The headline experiment we wanted to run — KGSL-style WPTR-polling
mode — is implemented but **hangs at ME_INIT** (`rptr/wptr=0/15`,
endless `recover_worker` loop). Details below.

## What's confirmed reproducible right now

Mainline kernel `cb28e40e789a`, mesa `e57fca6de2`. Everything default
unless noted. From a fresh `sysrq b` boot:

```
params: rbbm_poll=Y wptr_poll=N wptr_delay=0
unique samples: 8
MMU faults during run: 0
hangchecks during run: 0
hash freq:
   13 e3de0ab8
   13 9da287bb
   13 6ef9de51
   13 5adc3160
   12 9bbe68da
   12 626387c3
   12 1ab7f47e
   12 19e31a86
cycle pattern (first 24):
ABCDEFGH
ABCDEFGH
ABCDEFGH
```

The same 8-hash set survives across reboots (we've seen this set
twice now, both with `5adc3160` at 1/8). So the 8 cycle slots are
deterministic state that survives `sysrq b` reboots.

## RBBM_STATUS poll: completely decorative

The poll added in `8f3e3677` did:

```c
while (gpu_read(REG_A2XX_RBBM_STATUS) & A2XX_RBBM_STATUS_GUI_ACTIVE)
    cpu_relax();
```

before the WPTR write. We thought GUI_ACTIVE was the umbrella "GPU
is doing graphics" bit (it's used in a2xx_debugfs.c the same way).

A/B with `a2xx_rbbm_poll_enable=Y/N` gives **identical** distributions
(both 8-cycle, 12-13 each, same hashes). So the poll either:

* exits immediately because GUI_ACTIVE clears before the CPU returns
  from `gpu_write()` of the previous batch's CP_INTERRUPT, or
* the race is *not* "GUI is busy with previous batch" — it's
  something downstream of GUI clearing.

Worth your input on which other RBBM_STATUS bits to spin on if you
have a theory. Candidates we haven't tried:
- `RBBM_STATUS_RB_CNTX_BUSY` — RB context busy
- `RBBM_STATUS_SQ_CNTX0_BUSY | SQ_CNTX17_BUSY` — SQ context
- `RBBM_STATUS_MH_BUSY | MH_COHERENCY_BUSY` — memory hub
- `RBBM_STATUS_TC_BUSY` — texture cache

## wptr_delay=10000 regression

Update 11 had this as the most reliable single-knob "cycle collapse"
result. Today, with the same default settings on a kernel built
through Yocto from `cb28e40e789a`, the same 8-cycle persists with
`wptr_delay=10000` set. The udelay IS being executed (we've
instrumented it), but the cycle remains.

We don't have a clean explanation. The kernel has additional debug
patches (RBBM poll, the new module params) but none that should
materially change scheduling/timing on the wptr_delay path. The
mesa is the same SHA. Userspace stack is rebuilt by Yocto into
new packages.

If you have a hypothesis for what could make a 10 ms CPU spin
suddenly become non-effective, we're listening. (Compositor
preemption changing the per-submit time profile? Some workqueue
that used to run during the spin no longer running? A clock-tree
change?)

## WPTR-polling mode patch: hangs at ME_INIT

Implementation matches KGSL `kgsl_ringbuffer.c` setup:

* Added `volatile uint32_t wptr` field to `struct msm_rbmemptrs`.
* In `a2xx_hw_init()`, when `a2xx_wptr_poll_enable=true`:
  - Write `REG_AXXX_CP_RB_WPTR_BASE = lower_32_bits(rbmemptr(rb, wptr))`
  - Write `REG_AXXX_CP_RB_WPTR_DELAY = a2xx_wptr_poll_delay` (default 0)
  - OR `AXXX_CP_RB_CNTL_POLL_EN` into the existing RB_CNTL value
* In `a2xx_submit()` and `a2xx_me_init()`, before `adreno_flush()`:
  ```c
  ring->memptrs->wptr = get_wptr(ring);
  wmb();
  outer_sync();   /* CONFIG_OUTER_CACHE=y on this platform */
  ```

What happens when we enable `a2xx_wptr_poll_enable=Y` and trigger a
GPU reset (which re-runs `a2xx_hw_init` → `a2xx_me_init`):

```
[drm:adreno_idle] *ERROR* timeout waiting to drain ringbuffer 0
                   rptr/wptr = 0/15
[drm:hangcheck_handler] *ERROR* gpu lockup rb 0
[drm:recover_worker] *ERROR* hangcheck recover!
[drm:adreno_idle] *ERROR* timeout waiting to drain ringbuffer 0
                   rptr/wptr = 0/15
... endless loop, hw_init seq=2, 3, ... 162, ...
```

The CP never advances rptr past 0. The ME_INIT 15-dword packet
sits forever in the ring. Two interpretations:

* The CP isn't actually polling our `WPTR_BASE` location.
* The CP is polling but reads 0 (the initial value) because our
  cache-flush isn't reaching whatever memory the CP samples.

`outer_sync()` is real on this platform (`CONFIG_OUTER_CACHE=y`,
`CONFIG_CACHE_L2X0=y`). So our barrier should drain L1+L2 to
coherent memory.

Hypotheses we considered:

1. **Address encoding**: WPTR_BASE in the XML has no bitfield
   breakdown (just a 32-bit register). KGSL writes the gpuaddr
   directly. RPTR_ADDR has a `shr=2` bitfield, but WPTR_BASE
   doesn't appear to. This is what we did.

2. **MMU not yet active for CP**: At hw_init time, MMU_CONFIG is
   programmed but is the CP's read path already routed through it?
   If not, the CP would access *physical* memory at the value we
   wrote into WPTR_BASE, which would be invalid.

3. **Cache layer the CP reads from**: ARM v7 CPU writes go through
   L1 → L2 → coherent SoC memory. CP reads via AHB. `outer_sync()`
   should flush PL310 L2. But maybe there's a write buffer or
   another stage we're not draining.

4. **POLL_EN semantics**: Maybe POLL_EN means "ONLY read from
   memory, ignore the WPTR register write". If our memory isn't
   reaching the CP, the register write also gets ignored, so the
   CP just sees 0.

5. **Need RPTR_ADDR programmed too**: KGSL also programs
   `REG_CP_RB_RPTR_ADDR` to a memory location for CP to write rptr
   into. We don't (mainline reads RPTR via register).
   Maybe POLL_EN requires *both* memory hand-shakes to be wired up?

We haven't been able to disambiguate from the device side. Our test
binary still produces hashes when wptr_poll is enabled, but they're
nonsense (one of the existing 8-cycle hashes — probably a stuck
framebuffer because the CP is dead).

## Direct asks (in priority order)

1. **WPTR-polling failure mode**: which of (1)-(5) above is most
   likely, and what's the next concrete check? Reading
   `RB_RPTR/RB_WPTR/RBBM_STATUS` immediately after enabling POLL_EN
   would tell us if the CP is even seeing the register write — but
   we'd want to know what to write into the WPTR_BASE address to
   prove the path. Maybe set memptrs->wptr = 0xDEADBEEF and check
   whether the CP latches that as wptr?

2. **Other RBBM_STATUS bits to poll on**: GUI_ACTIVE is decorative.
   Which of the more specific bits (RB_CNTX, SQ_CNTX0/17, MH,
   MH_COHERENCY, TC, CP_NRT, CP_COHER) is most likely to actually
   be the "settle window" that matters?

3. **wptr_delay=10000 regression hypothesis**: anything in your
   model of the SoC that would make a 10 ms CPU spin become
   ineffective when it used to work?

4. **Still worth attempting power-button cold-cold boot test
   from update 12?** With the cycle now reliably reproducible
   from `sysrq b` (warm) reboots, the "is the cycle stuck-state
   that needs power-off to clear?" question is less urgent — but
   would still tell us whether the cycle's *mechanism* is in
   warm-survivable state or not.

## Appendix: artifacts

* `/tmp/r100test/baseline/sample-<hash>.bin` × 8 on device — one
  3 MB capture per unique hash, plus `hashes.txt` with the full
  100-run sequence. Available for off-device byte-level diff if
  it would help your analysis.
