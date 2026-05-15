# Update 6 for Gemini: MH_ARBITER fix is real but partial

## What we did

Built a webOS-side regdump tool that uses the legacy KGSL ioctl
`IOCTL_KGSL_DEVICE_REGREAD` to read MMIO directly, then ran it on
HP TouchPad / Leia (Adreno 220, REV470) booted into webOS to capture
the **KGSL "good state"** snapshot of every MH register and the
MH_DEBUG_CTRL sweep (0..63).

Compared against our mainline freedreno hw_init dump on the same
hardware. One register differs:

```
                   webOS / KGSL    mainline freedreno
MH_ARBITER_CONFIG  0x07c86590      0x07c8e590     <-- bit 15 differs
                                                      ^IN_FLIGHT_LIMIT_ENABLE

MH_CLNT_INTF_CFG1  0x00032f07      0x00032f07     match
MH_CLNT_INTF_CFG2  0x00cf2f47      0x00cf2f47     match
MH_INTERRUPT_MASK  0x00000007      0x00000007     match
AXI_ERROR          0               0              match
INT_STATUS         0               0              match
MH_DEBUG[0..63]    (32 mostly-zero values)        ALL match
```

**KGSL kgsl_yamato.c:60** confirms the design intent:
```c
#define KGSL_CFG_YAMATO_MHARB \
    (...
     | (0 << MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT_ENABLE__SHIFT) \
     | (0x8 << MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT__SHIFT) \
     ...)
```

The IN_FLIGHT_LIMIT field is filled with 8 but the ENABLE bit is
deliberately off — the limit is unenforced. Same value used for
both Yamato (a20x) and Leia (a22x); no chip_id branching in KGSL.

Mainline freedreno enables it. We removed the enable.

## What happened

**Live verification**: kernel diagnostic dump of MH_DEBUG[31] (which
appears to be a debug-bus readback of MH_ARBITER_CONFIG) changed
from `0x57c8e590` to `0x57c86590` after the fix — confirming the
register write is sticking.

**100-cap test**:
```
unique samples: 8
MMU faults:     0
hash freq:
   13 e5644c73...     <-- NEW broken hash (was fb0772c9 before fix)
   13 5adc3160...     <-- STILL the correct cold-boot hash, STILL 13/100
   13 24ed6c24...     <-- NEW (was f197cb26)
   13 03dee03a...     <-- NEW (was 37242f10)
   12 e93d10aa...     <-- NEW (was aae50ced)
   12 bebc09c6...     <-- NEW (was 91666260)
   12 53fb83a8...     <-- NEW (was 4b895c7a)
   12 07c687fb...     <-- NEW (was 3584c308)
cycle: ABCDEFGH ABCDEFGH ABCDEFGH ...
```

* Period-8 unchanged (still 4×13 + 4×12 = 100, still ABCDEFGH order).
* "Good" hash 5adc3160 still appears at exactly 1/8 frequency.
* All seven "broken" hashes are completely different bit patterns
  from before. **The MH change is real and affects rendering** — but
  it's not the cycle source.

## What this proves

* IN_FLIGHT_LIMIT_ENABLE is NOT the cycle-source register. We cleared
  the lone MH register difference vs KGSL, and the cycle persisted.
* The MH arbiter difference DOES affect what the broken outputs look
  like (different hashes prove this), so the fix is meaningful — just
  not sufficient.
* All readable MH state at hw_init time matches KGSL. So the period-8
  source is either:
  - In MH state we can't read at hw_init (per-submit dynamic state)
  - In a different block entirely (CP, SQ, VFE, MDP-side)
  - In something software-side (kernel scheduler, page allocation
    pattern, msm_gem object reuse)

## Visual side-channel

I have PNGs of all 8 unique outputs from before-fix and after-fix in
`reports/fb-captures/`. Two channels of differential observation:

1. **before vs after** — same cycle position, different broken pattern
2. **across positions in one set** — what's similar/different among the
   7 broken outputs

Want me to share the actual PNGs (or pixel-diff stats) so you can
look at what kind of corruption we're dealing with? My read so far:
the broken outputs all share a "missing R or B channel, never G"
fingerprint with stale-tile artifacts — suggests something specifically
operating on per-component routing, not generic memory corruption.

## What we still know

The cycle period is 8. The cycle is per-PROCESS-spawn, not per-frame
within a process (test is 100 separate `gl-cap-and-regdump` invocations
each opening a fresh DRM fd, drawing one triangle, glReadPixels-ing,
exiting). The kernel-side state we've audited:

| Resource | Count |
|---|---|
| GPU rings | 1 (`adreno_gpu_init(... 1)`) |
| MH_MMU context banks | 1 (single `PT_BASE`) |
| SoC IOMMU GPU CB | 2 (DT `qcom,ncb`) — A2XX doesn't use this anyway |
| SQ contexts (per RBBM_STATUS) | 18 (CNTX0..CNTX17) |
| MH ARBITER IN_FLIGHT_LIMIT | 8 (just disabled with our fix) |

The only "8" left in the audit is the IN_FLIGHT_LIMIT field's value,
but the enforce bit is off so the field is dead.

## Asks for you

1. **Where else in the A2XX / MSM8660 graphics pipeline is "8" likely
   to surface?** Things we haven't audited: per-DRM-fd msm_gem object
   IDs, MDP4 internal pipe scheduling, the LSM compositor's window-
   surface buffer rotation, ICC fabric arbitration window, kernel
   scheduler priority tier bound to fence seqno mod 8.

2. **Is the fact that the broken hashes CHANGED meaningful diagnostically?**
   If the cycle source were entirely in some downstream block (e.g.,
   MDP), changing MH state shouldn't affect rendering at all. The
   fact that it does means our IN_FLIGHT_LIMIT change is altering
   the *content* of the broken outputs. That suggests there's still
   *some* MH involvement — perhaps a different MH-related bit we
   haven't tried.

3. **Are there A22X-specific registers in the MH block we're missing?**
   We covered the documented set (CONFIG1, CONFIG2, ARBITER, MMU_*,
   INTERRUPT_*, AXI_ERROR). MH_DEBUG_CNTL[0..63] dump showed mostly
   zeroes but slot 31 contained ARBITER_CONFIG echo — slot 22 had
   `0x90000002`, slot 23 `0x20000002`, slot 27 `0x00047fff`,
   slot 28 `0x0020003f`, slot 29 `0x8d100947`, slot 31 `0x57c86590`.
   These are mostly stable across hw_inits but could be writable
   debug registers we're missing.

4. **Should we add a per-submit MH state dump** (not per-hw_init) to
   see if any MH register CHANGES between batches within one hw_init
   session? hw_init only fires every ~500ms while submits happen
   every ~50ms, so per-submit visibility is the next instrumentation
   step if hw_init-time state genuinely doesn't cycle.

The IN_FLIGHT_LIMIT fix is going in regardless — it matches KGSL and
is provably the correct config for this hardware. But the cycle hunt
continues. What's your next priority?
