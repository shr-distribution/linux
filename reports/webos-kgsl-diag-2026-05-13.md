# webOS legacy kernel KGSL ISSUEIBCMDS diagnostic — 0x0ee2 cycle counter

**Date:** 2026-05-13
**Kernel:** patched 2.6.35-palm-tenderloin (KGSL printk on every ISSUEIBCMDS)
**Boot method:** `novacom boot mem:// < uImage` directly (no flash)

## TL;DR

The legacy webOS kernel was patched to read 0x0ee2 before and after every `IOCTL_KGSL_RINGBUFFER_ISSUEIBCMDS`. Booted on real hardware. Captured 558 submits during webOS home-screen rendering + cap-binary GL render. **The data overturns several assumptions about 0x0ee2.**

## Cap-binary's 16 ISSUEIBCMDS (pid=3393)

Exact 16 submits matching webOS strace observation. Same triangle render Mesa does in 1 ioctl, webOS splits into 16:

| ts | nibs | 0x0ee2 before | 0x0ee2 after | Notes |
|----|------|---------------|--------------|-------|
| 3689 | 1 | 0x7ff58329 | 0x7ff2d079 | **0x7f namespace** |
| 3693 | 1 | 0x15060602 | 0x15060602 | idle, no change |
| 3695 | 1 | 0x1500c0c2 | 0x1503c3c2 | 0x15 jump |
| 3697 | 1 | 0x1503c3c2 | 0x1506c6c2 | 0x15 jump |
| 3699 | 1 | 0x1506c6c2 | 0x1506c6c2 | idle |
| 3701 | 1 | 0x1506c6c2 | 0x1506c6c2 | idle |
| 3703 | 1 | 0x15068682 | 0x15014142 | 0x15 jump |
| 3705 | 1 | 0x15014142 | 0x15044442 | 0x15 jump |
| 3707 | 1 | 0x15044442 | 0x15044442 | idle |
| 3709 | 1 | 0x15044442 | 0x15044442 | idle |
| 3711 | 1 | 0x15074742 | 0x15040402 | 0x15 backward |
| 3713 | 1 | 0x1501c1c2 | 0x1501c1c2 | idle |
| 3788 | 1 | 0x7ff29039 | **0x1501c1c2** | **0x7f → 0x15 cross-namespace jump** |
| 3953 | 1 | 0x15078782 | 0x15078782 | idle |
| 3956 | 1 | 0x15078782 | 0x15078782 | idle |
| 3958 | 1 | 0x15078782 | 0x15078782 | idle |

cap.bin md5 = `abda39bc2c6c69367a65f2dc685ff20e` — webOS rendered output.

## Global statistics (558 submits from all userspace)

```
High-byte distribution of 0x0ee2_after:
  0x15  : 312  (56%)
  0x7f  : 148  (27%)
  0x29  : 65   (12%)
  0x7d  : 15   (2.7%)
  0x69  : 6
  0x2b  : 4
  0x6b  : 3
  0x3f  : 3
  0x3d  : 2

Same vs changed:
  before == after : 266  (48%)
  before != after : 292  (52%)
```

## Decisive new findings

### 1. 0x0ee2 is NOT a monotonic counter

Mainline freedreno's hypothesis was that 0x0ee2 monotonically advances by +0x00020200 per submit, with period 16.  webOS data shows:
- **Backward transitions** exist (e.g., 0x15074742 → 0x15040402)
- **Cross-namespace jumps** (e.g., 0x7ff29039 → 0x1501c1c2)
- **No-op submits** (50% of all submits) leave the register unchanged

So 0x0ee2 is a **multi-mode state register**, not a simple cycle counter.

### 2. Multiple "namespaces" in 0x0ee2

The high byte takes ~9 distinct values across submits.  Most common:
- **0x15** (most renders) — appears to be the "rasterization idle" state
- **0x7f** (binner active?) — large fraction of submits land here
- **0x29** (third mode — possibly resolve/copy work)

Mainline only ever exercises the 0x15 namespace because it issues only "raster-and-resolve" type submits.  webOS exercises 0x7f, 0x29, 0x7d, 0x69, 0x2b, 0x6b, 0x3f, 0x3d additionally — those are likely binner, dispatch, fence, etc., types of submits we never issue.

### 3. The 0x15 sub-states match mainline's period-16 cycle exactly

All 16 of mainline's observed values are present in webOS data:
```
0x15000002, 0x15004042, 0x15008082, 0x1500c0c2,
0x15010102, 0x15014142, 0x15018182, 0x1501c1c2,
0x15020202, 0x15024242, 0x15028282, 0x1502c2c2,
0x15030302, 0x15034342, 0x15038382, 0x1503c3c2,
0x15040402, 0x15044442, 0x15048482, 0x1504c4c2,
0x15050502, 0x15054542, 0x15058582, 0x1505c5c2,
0x15060602, 0x15064642, 0x15068682, 0x1506c6c2,
0x15070702, 0x15074742, 0x15078782, 0x1507c7c2
```

But the webOS data has **MORE 0x15 sub-states** (32 distinct values observed), not just 16!  Mainline only sees 16 because mainline only emits one type of work item.

### 4. Cap-binary on webOS ends at 0x15078782 (idle)

After the cap binary's 16 submits, 0x0ee2 settles at `0x15078782` and stays there.  This is the **idle / between-frames** state for our cap workload.  The previous mainline-observed values were SNAPSHOTS at this "idle" point, not the mid-render state.

### 5. rbbm_before always reads 0

All 558 submits show `rbbm_before=0x00000000`.  Could indicate:
- We read while GPU is clock-gated (more likely)
- Or webOS keeps GPU mostly idle between submits

## Implications for the period-16 fix

The "period-16 cycle on Mesa" is now better understood as:

> Mesa's single-IB-per-render path lands the GPU in a specific 0x15-namespace slot.  Each render advances by +0x00020200 to the next slot.  The renderer's appearance depends on which slot we land in.  webOS avoids this by issuing 16 sub-submissions per render, exercising MULTIPLE slots and ending at slot 0x15078782 — which is one of the slots that produces correct output.

The fix is NOT "reset 0x0ee2 to a magic value".  The fix is **issue multiple sub-submissions per render** that collectively drive 0x0ee2 through enough states to land at a correct one.  This was our **multi-flush v3** hypothesis — but our implementation failed because the dummies we issued didn't actually advance 0x0ee2 (CP_NOP / CACHE_FLUSH / DEALLOC events don't move the slot pointer).

**The submissions that webOS issues are NOT all dummy work** — they are actual rendering sub-pieces.  Some are binning passes (0x7f namespace), some are state setup, some are tile rendering, some are resolves.  Each contributes a meaningful transition.

## Next path

Either:

1. **Decode what each of webOS's 16 sub-submissions DOES** (the IB content).  If we can identify "binner submit" vs "tile submit" vs "resolve submit", we can structure Mesa's cmdstream to match.

2. **Per-tile split in Mesa** — make each tile its own MSM_SUBMIT.  Vendor patterns suggest each tile = ~2-3 ISSUEIBCMDS.  For a small framebuffer (1 tile) that gives 2-3 submissions = 1/8 of webOS's 16.  Could be enough to land at the right slot, or might still cycle.

3. **Explicit per-stage submissions** — break Mesa's submission into (a) binning IB submit, (b) per-tile renderprep + draws submit, (c) resolve submit per tile, (d) final present submit.  Match webOS's pattern functionally.

## Repo

- Patched legacy kernel source: `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/gpu/msm/kgsl.c`
- Built uImage: `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/arch/arm/boot/uImage`
- dmesg dump (this run): `/tmp/webos-full-dmesg2.log`
- cap.bin (webOS render): `/tmp/webos-cap.bin` (md5 abda39bc...)
- regs.txt (webOS register dump): on device at `/media/internal/regs.txt`

---

## Round 2 — IB-content decode (2026-05-13)

Extended kernel patch to also dump first 32 dwords of each IB.  `/var/log/messages` on device captures the data persistently.

### Submit-size distribution (378 submits captured)

| sz (dwords) | count | role |
|-------------|-------|------|
| 153 | 202 (53%) | dominant per-frame draw |
| 156 | 60 | variant |
| 2 | 36 | pure `CP_WAIT_FOR_IDLE` sync |
| 143 | 26 | small draw |
| 1473 | 13 | state init |
| 165 | 11 | mid-size draw |
| 1185 | 10 | shader load + state init |
| 744 | 5 | mid-large |
| 146 | 5 | small draw |
| Others | 1-2 each | rare |

### IB-class decoding (verified)

```
sz=2 (sync):
  +00: c0002600 00000000  -- CP_WAIT_FOR_IDLE
  -- 0x0ee2 NEVER changes for these submits

sz=153 (draw, 0x0ee2 changes):
  +00: c0012d00 00040202 00001c07 c0022d00  -- CP_SET_CONSTANT (state)
  +10: 00040081 00000000 03000400 c0042d00
  +20: 00040303 411aaaab 3f800000 40e00000
  +30: 3f800000 c0042d00 0004010f 44000000
  +40: 44000200 c3c00000 43c00400 c0012d00
  +50: 00040312 0000ffff c0002600 00000000  -- CP_WAIT_FOR_IDLE mid-stream
  +60: c0012d00 00040200 0070079c c0012d00
  +70: 00040205 00090000 c0012d00 00040206
  -- (more state + DRAW_INDX further in the 153 dwords)

sz=1185 (state setup, 0x0ee2 UNCHANGED):
  +00: c0042d00 00000080 3f800000 00000000  -- CP_SET_CONSTANT
  ...
  +50: c01f2b00 00000000 0000001e 02556003  -- CP_IM_LOAD_IMMEDIATE (shader load)
  +60: 00001000 c2000000 00001009 00001000  -- 32 dwords of ALU instructions
  +70: c4000000 0000000a 00002000 00000000
  -- shader code load + register state, no DRAW_INDX
```

### PM4 opcode behavior with respect to 0x0ee2

| Opcode | Name | Moves 0x0ee2? |
|--------|------|---------------|
| 0x22 | CP_DRAW_INDX | **YES** (confirmed earlier on mainline) |
| 0x26 | CP_WAIT_FOR_IDLE | No |
| 0x2b | CP_IM_LOAD_IMMEDIATE | **No** (sz=1185 IBs with this don't advance counter) |
| 0x2d | CP_SET_CONSTANT | **No** (most state writes don't advance counter) |
| 0x2f | CP_SET_BIN_DATA | Untested |

### Architectural insight

**webOS splits work into multiple submits:**
1. **State-setup submits** (sz=1185/1473/etc.) load shaders and register state.  No DRAW_INDX → no 0x0ee2 advance.
2. **Draw submits** (sz=143/153/156) issue actual drawing.  Contains DRAW_INDX → advances 0x0ee2.
3. **Sync submits** (sz=2) inject CP_WAIT_FOR_IDLE barriers between work.  No state change.

**Mainline Mesa puts EVERYTHING into a single submit** (state + shader load + draws + resolves + sync).  All in one mega-IB.

This means the per-draw counter advance pattern IS the same in both — what differs is whether the GPU has time to settle between renders.  webOS's CP_WAIT_FOR_IDLE-only sync submits create natural GPU-idle boundaries that the slot allocator may respect.

### Cap binary attribution

Only **3 ISSUEIBCMDS from pid=3453 (the cap binary directly)** appeared in /var/log/messages:
```
ts=2982: before=0x15014142 after=0x15014142  (no change)
ts=2985: before=0x15014142 after=0x15014142  (no change)
ts=2987: before=0x15014142 after=0x15014142  (no change)
```

All three are no-op (probably EGL surface init / SwapBuffers stubs from the cap binary's process).

The **actual rendering happens in pid=2558 (LunaSysMgr / SurfaceManager)** — the cap binary's GL calls are proxied to the system compositor.  webOS's architecture is GL-via-compositor, not direct-GL.

This means the 16 submits-per-render isn't strictly per-cap-binary-render — it's how the compositor renders any frame (including cap-binary's contribution).

### Next-step candidates (priority-ordered)

1. **Mesa per-tile / per-stage submit split**: Match webOS's pattern of "state-setup submit → draw submit → sync submit → resolve submit" instead of one mega-submit.  Per-tile = 2-3 submits.  For 1-tile framebuffer (small cap), that's 2-3 submits/render, not 16.  But the principle of "split state setup off from draws" is testable.

2. **Sync-only submits between renders on Mesa**: Mesa already emits CP_WAIT_FOR_IDLE inside its IB.  Maybe ALSO emitting separate sz=2 sync-only submits BETWEEN renders gives the GPU enough time to drain slot state.

3. **Decode the cap-binary's 16 submits from a FRESH run** with the slow `/var/log/messages` collection — we can identify exactly which one contains the user's triangle DRAW_INDX, and analyze the surrounding state. (Needs re-run with empty messages.log + isolation of cap binary invocations.)
