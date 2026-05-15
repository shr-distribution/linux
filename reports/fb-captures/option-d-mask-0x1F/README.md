# Option D test — RBBM_SOFT_RESET mask 0x1F

Kernel: tenderloin/6.18/upstream-patches @ `b0025a750c54`
Mesa: 26.1.0+git-e57fca6de2 with patches 0001/0002/0017/0040/0045/0046

`a2xx_pulse_gfx3d_reset()` called from `a2xx_submit` right before
`adreno_flush()` writes WPTR:

```c
gpu_write(gpu, REG_A2XX_RBBM_SOFT_RESET, 0x0000001F);
udelay(5);
gpu_write(gpu, REG_A2XX_RBBM_SOFT_RESET, 0x00000000);
udelay(5);
```

Per Gemini's update-24 reply, mask `0x1F = BIT(0..4) = VGT | PA/VPC
| SQ | SX | TC`. Intentionally avoiding BIT(7) = MH and BIT(8) =
CP.

## Result: NO EFFECT

Phase A (knob=Y, pulse fires before every submit) and Phase B
(knob=N, pulse disabled) produced **bit-identical** captures:

| Phase A unique hash | Phase B unique hash | Same channel means | Pixel diff |
|---------------------|---------------------|--------------------|------------|
| sample-5adc3160     | sample-5adc3160     | R=34.27 G=34.28 B=34.21 A=103.37 | **0**      |
| sample-9e25589e     | sample-9e25589e     | R=11.59 G=3.54 B=13.56 A=103.37  | **0**      |
| sample-73bb37bb     | sample-73bb37bb     | R=34.92 G=33.22 B=26.23 A=103.37 | **0**      |

The 8-cycle is unchanged. The same 8 hashes with the same 12-13
distribution appear in both phases. Phase A is byte-identical to
Phase B for any given hash.

## Conclusion

The `0x1F` mask is effectively a no-op. Either:

1. **Gemini's bit map is wrong** for A2XX (it conflicts with KGSL's
   comment "Only reset CP block" applied to value `0x00000001`,
   which implied bit 0 = CP, not VGT).

2. **The CPU MMIO write is being rejected** because Mesa's
   ME_INIT has enabled PROTECTED_MODE which blocks RBBM_SOFT_RESET
   writes from the CPU side. Resets would need to come from the
   CP via PM4 packets.

3. **The 8-cycle's state machine isn't in any of VGT/PA/SQ/SX/TC**
   and is instead in something outside the 0x1F mask (RB, MH, GMEM
   resolve queue, etc.).

## Files

`phase-A-pulse/` and `phase-B-no-pulse/` each contain 8 unique
sample bins + their PNG renders. Channel means and per-hash pixel
diffs show they are bit-identical pairs.

The 8 baseline hashes:
* 070bdc57 — R=11.9 G=12.5 B=32.5
* 259d419d — R=5.0  G=4.0  B=40.1
* 48845819 — R=6.4  G=6.6  B=20.3
* **5adc3160 — R=34.3 G=34.3 B=34.2** (the channel-mean-correct render)
* 73bb37bb — R=34.9 G=33.2 B=26.2
* 9e25589e — R=11.6 G=3.5  B=13.6
* acb14db9 — R=12.8 G=1.4  B=24.0
* ccb21b89 — R=29.7 G=24.0 B=11.8

These match the original baseline 8 hashes from previous tests
(before any of our power-cycle attempts).

## Next steps

1. **Verify register write takes effect**: add a readback after
   the write to confirm RBBM_SOFT_RESET actually receives `0x1F`.
   If readback is `0`, protected mode is blocking us.
2. **Try alternative masks**: make the mask a module param for
   live A/B testing without rebuilding.
3. **If protected mode IS blocking writes**: emit the reset via a
   PM4 packet inside the ringbuffer rather than via CPU MMIO. The
   CP can write any register because it operates inside the
   protection domain.
