# Fork D VSC pipe BO dump diagnostic (2026-05-11)

## Setup

- Kernel: `g6044076c975e` (debugfs additions)
- Mesa: `7cb244d6d1bc...` (Fork D + 0094 dump diagnostic, env-controlled)
- Tool: `gl-cap-and-regdump-mainline` with double-render mode (runs triangle twice
  per invocation when `FD2_VSC_DUMP=1`)

## Test: 30 invocations with `FD2_VSC_DUMP=1`

```
$ FD2_VSC_DUMP=1 /tmp/gl-cap-and-regdump-mainline   # × 30
```

For each invocation, save:
- `cap.bin` md5 (rendered framebuffer hash)
- `vsc_pipe<P>_batch1.dump` (first 4KB of each VSC pipe BO, dumped by Mesa
  at start of batch 2's fd2_emit_tile_init — i.e. captures batch 1's binner output)

## Result 1: Cycle drops from period-8 to period-2

Only **two unique cap.bin hashes** seen across 30 runs:
- `b8691119eda2` (15 hits)
- `2ca33faf8182` (15 hits)

Neither is the golden `5adc3160`. Single-render Fork D produced an 8-cycle.
Double-render produces a 2-cycle, perfectly alternating.

## Result 2: All VSC pipe BO dumps are 100% zero

```
$ for p in 0..7; do tr -d '\0' < vsc_pipe${p}_batch1.dump | wc -c; done
# every pipe: 0 non-zero bytes (of 4096)
```

```
$ md5sum vsc_pipe*_batch1.dump
620f0b67a91f7f74151bc5be745b7110  vsc_pipe0_batch1.dump
620f0b67a91f7f74151bc5be745b7110  vsc_pipe1_batch1.dump
...
620f0b67a91f7f74151bc5be745b7110  vsc_pipe7_batch1.dump
```

All 8 pipes contain the same all-zero content. Confirmed by `cmp` across pipes
and across the two cycle hashes — every dump is byte-identical.

`620f0b67a91f7f74151bc5be745b7110` is the md5 of 4096 zero bytes.

## Interpretation

The A22X binner is **engaged but starved**. Even though our Fork D prelude
writes:
- `0xC00 = 1` (VSC enable)
- `LRZ_VSC_CONTROL = 3` (engage binner)
- VSC_BIN_SIZE + VSC_PIPE[0..7] config

— the binner is not producing any visibility-stream output. The "hw binning"
plumbing is set up but inert.

This explains why Fork C v2 / Fork D / kernel cold-start pulse all failed
to break the cycle: they were never actually engaging real binning. The
prelude tickles the binner state machine enough to shift cycle composition
(per Fork D's 4/7 hash changes) but doesn't make the binner actually do
visibility computation.

The period-8 vs period-2 difference between single- and double-render is
also informative — it shows the cycle responds to **submit count**, not
to register state.

## Hangs in double-render mode

dmesg shows hangcheck recover events during this 30-run test:
- ~2 hangcheck events
- Each recovers cleanly (next submit succeeds)
- offending task: gl-cap-and-regdump-mainline

The double-render + fd_bo_cpu_prep wait inside Mesa is more stressful than
single-render and exposes a hang race. Doesn't change the conclusion.

## Strategic conclusion

**The binner is not doing real binning work.** Adding registers won't fix
this — the binner needs a binning shader pass (a VS-only stripped shader
emitted in a separate IB) to actually compute and write visibility data.

That's Fork A/B (full hw binning port) territory. Without it, the binner
is permanently inert and the cycle persists.

Remaining viable options:
1. **Fork A/B full port** (1-2 weeks of shader-compiler work)
2. **Ship 12.5% baseline** (single-render Fork D / Fork C v2)

The BO dump definitively rules out the "fix it with a register" path.
