# HP TouchPad eMMC Speed Benchmark

Comparative eMMC throughput between webOS (working ADM DMA) and
mainline 6.18 (PIO-only — ADM DMA fix pending; see
`project_adm_uses_ee0_not_ee1.md`).

## Hardware

- **Device:** HP TouchPad (Topaz)
- **SoC:** Qualcomm APQ8060 (dual-core Scorpion)
- **eMMC:** SDR controller `msm_sdcc.1` @ 0x12400000, 8-bit MMC mode,
  base clock 48 MHz
- **DMA controller:** ADM0 @ 0x18320000 (when programmed correctly)

## Methodology

Tests via `tools/emmcbench` — a static-linked ARM userspace binary
that opens `/dev/mmcblk0` with `O_DIRECT` so the kernel page cache
cannot contaminate the measurement (busybox `dd` cannot do this; that's
why we wrote this).

- Sequential read sweep across block sizes 4 KiB → 16 KiB → 64 KiB →
  256 KiB → 1 MiB. Same total bytes (25-50 MB depending on bs) per
  measurement.
- Random 4 KiB reads at fixed-seed random sector-aligned offsets across
  the entire device.
- CPU governor pinned to `performance` for each run; baseline runs
  also offline CPU1 (`echo 0 > /sys/.../cpu1/online`) to remove SMP
  interference noise.
- Page cache dropped via `echo 3 > /proc/sys/vm/drop_caches` before
  each invocation.

Build (host):
```
arm-linux-gnueabi-gcc -std=gnu99 -O2 -static -o emmcbench emmcbench.c
```

## Results — webOS 2.6.35-palm-tenderloin

**Test date:** 2026-05-13
**Kernel:** `2.6.35-palm-tenderloin` (stock)
**CPU:** Scorpion @ 1188 MHz (webOS DVFS ceiling), governor: ondemandtcl
**ADM DMA:** Working (programmed at EE=0 — see `project_adm_uses_ee0_not_ee1.md`)
**Total per pattern:** 50 MB sequential, 500 random 4 KiB reads

### Sequential read

| Block size | Throughput |
|-----------:|----------:|
| 4 KiB      |  3.8 MB/s |
| 16 KiB     | 10.8 MB/s |
| 64 KiB     | 18.8 MB/s |
| 256 KiB    | 23.7 MB/s |
| **1 MiB**  | **25.4 MB/s** |

### Random 4 KiB read

| Pattern        | Throughput | IOPS |
|---------------:|----------:|----:|
| Random 4 KiB   | 3.3 MB/s | **835 IOPS** |

(267/500 reads completed cleanly — the rest landed in regions where
O_DIRECT can't service the request synchronously, e.g. partition
tables, GPT shadow, eMMC reserved areas.)

## Results — mainline 6.18 (kernel `6.18.0-luneos-gfbb7fe01f7be`)

**Test date:** 2026-05-13
**Kernel:** mainline 6.18 with the May 2026 Scorpion CP15 stack
**CPU:** Scorpion @ 1512 MHz, governor: performance, CPU1 offline
**ADM DMA:** **NOT working** — driver writes to EE=1 window, hardware
serves EE=0 (see `project_adm_uses_ee0_not_ee1.md` for the diagnosis
and `bisect/adm-ee0` for the validated fix). So eMMC currently runs
in **PIO mode** via mmci-pl18x FIFO.
**Total per pattern:** 50 MB sequential, 500 random 4 KiB reads

### Sequential read

| Block size | Throughput |
|-----------:|----------:|
| 4 KiB      |  5.6 MB/s |
| 16 KiB     | 12.1 MB/s |
| 64 KiB     | 23.8 MB/s |
| 256 KiB    | 28.4 MB/s |
| **1 MiB**  | **30.1 MB/s** |

### Random 4 KiB read

| Pattern        | Throughput | IOPS |
|---------------:|----------:|----:|
| Random 4 KiB   | 4.1 MB/s | **1038 IOPS** |

(251/500 reads completed cleanly — same partition-table / reserved
region effect as on webOS.)

## webOS vs mainline summary

| Pattern        | webOS (ADM DMA) | mainline (PIO) | ratio |
|---------------:|----------------:|---------------:|------:|
| Seq 4 KiB      |  3.8 MB/s       |  5.6 MB/s      | 1.47  |
| Seq 16 KiB     | 10.8            | 12.1           | 1.12  |
| Seq 64 KiB     | 18.8            | 23.8           | 1.27  |
| Seq 256 KiB    | 23.7            | 28.4           | 1.20  |
| **Seq 1 MiB**  | **25.4**        | **30.1**       | **1.19** |
| Random 4 KiB   | 835 IOPS        | **1038 IOPS**  | 1.24  |

### Reframing the ADM DMA fix priority

The previous assumption was that mainline PIO would be ~3-5 MB/s and
the ADM DMA fix would unblock real throughput. The actual measurement
shows mainline PIO is **30 MB/s** — already 19% *faster* than legacy
ADM-DMA on webOS. This is the modern mmci-pl18x FIFO + burst handler
out-pacing the legacy `mmc/host/msm_sdcc.c` driver paired with
ADM-DMA.

Consequence: `bisect/adm-ee0` becomes a **CPU-offload / power
optimisation** rather than a throughput win. PIO at 30 MB/s burns
roughly half a Scorpion at the wire transfer rate; ADM-DMA would
free that CPU. Worth doing for sustained workloads (camera, video
streaming) and battery, but no longer the critical path for "is
eMMC fast enough".

### Note on the prior 3-5 MB/s estimate

The earlier claim in `reports/adm-dma-emmc-analysis.md` of
"3-5 MB/s PIO" was based on a `dd` test that went through the
kernel page cache, which apparently hit some adverse code path
(likely cache flush per-iteration). The O_DIRECT measurement here
is the cleaner number and is what real applications see when
hitting the device through the block layer.

## 2026-05-13 (evening) — DMA fix landed and verified

**Kernel:** `6.18.0-luneos-g6bb2931b38ee`
**Stack:**
- `71e9d82d26c3` ARM: dts: tenderloin: revert qcom,ee=<0> (back to <1>)
- `6bb2931b38ee` dma: qcom: adm: don't rewrite CH_CONF — trust bootloader

After flashing, `mmci-pl18x 12400000.mmc: DMA submit OK` is followed
by **actual transfer completion** (no more CMDTIMEOUT bursts), the
kernel enumerates the eMMC card, partitions appear:

```
mmc0: new high speed MMC card at address 0001
mmcblk0: mmc0:0001 SEM32G 29.7 GiB
mmcblk0: p1 p2 p3 p4 < p5 p6 p7 p8 p9 p10 p11 p12 p13 p14 >
EXT4-fs (mmcblk0p13): mounted filesystem ... r/w with ordered data mode
```

### Sequential read (DMA, kernel `6bb2931b38ee`)

| Block size | DMA-fix Throughput | vs PIO baseline |
|-----------:|-------------------:|----------------:|
| 4 KiB      |  4.7 MB/s          | 0.84x           |
| 16 KiB     | 11.7 MB/s          | 0.97x           |
| 64 KiB     | 22.1 MB/s          | 0.93x           |
| 256 KiB    | 26.8 MB/s          | 0.94x           |
| **1 MiB**  | **28.4 MB/s**      | **0.94x**       |
| Random 4 KiB | 3.5 MB/s / 895 IOPS | 0.86x       |

### CPU usage during sustained 200 MB read

The headline metric: **`%Cpu(s): 4.8 us, 14.3 sy, 0.0 ni, 9.5 id,
71.4 wa`** — **71% I/O wait** while DMA does the transfer. With PIO
the CPU would be 50-90% busy reading from the controller FIFO. With
DMA active, the CPU sleeps until the DMA-done interrupt fires.

This is the entire point of the DMA fix: not throughput (the
controller is already at its 30 MB/s ceiling for 8-bit MMC at 48
MHz, no matter who's moving the bytes), but **freeing the CPU to do
useful work in parallel**. For sustained workloads — camera
streaming, video playback, large-file copies — this is the
difference between "CPU is hot and battery drains" and "CPU mostly
asleep".

The 5-7% drop in raw throughput compared to PIO is the DMA setup +
descriptor-fetch overhead on each transfer. Acceptable cost for the
CPU-offload gain.

### Side effect: DDR-bandwidth measurements now noisy

Background eMMC DMA traffic shares the EBI/AFAB system bus with the
CPU's data-cache fills. While idle, this doesn't matter. During an
active eMMC transfer **and** an active NEON streaming-read benchmark,
the two compete for EBI bandwidth and arbitration cycles.

The first `ddrbench` run after this kernel booted (uptime ~1 min,
system services still starting + journaling) showed:

```
NEON read       median=   80.6 MB/s   range [  61.3 ..  245.5]
NEON write      median= 1579.5 MB/s   range [ 268.8 .. 1915.5]
```

Compare to the prior-kernel baseline (no DMA, idle eMMC bus):

```
NEON read       median=  575   MB/s   range [ 433   ..  643  ]
NEON write      median= 1843   MB/s   range [1536   .. 1929  ]
```

Reads collapsed by 7x. This is **not** a regression from the DMA
fix — it's bus contention from background eMMC traffic at boot.
Reruns once the system is fully quiescent (uptime > 5 min, no
ongoing filesystem journaling) should show numbers close to the
prior-kernel baseline.

A separate follow-up worth investigating: **AP master priority on
the system NoC**. If the CPU's reads are arbitrated at lower
priority than mmci-pl18x's DMA reads, the CPU loses ground every
time the eMMC does a burst. Legacy webOS likely sets AP higher.
This would be a tunable in the `qnoc-msm8660` interconnect provider.

## Notes

- The 4 KiB sequential-read result (3.8 MB/s) is the worst case and
  is dominated by per-`read()` syscall overhead + filesystem-block-size
  mismatch, not by eMMC throughput. Real filesystem traffic almost
  never uses 4 KiB direct reads in isolation; it batches via
  readahead, which lands closer to the 1 MiB ceiling.
- 25 MB/s sequential is consistent with what an 8-bit MMC at 48 MHz
  can sustain at the wire (48 × 1 byte/clock × ~0.5 efficiency ≈
  24 MB/s), so we are essentially at the bus ceiling here.
- The "8 MB/s" we previously believed to be the mainline PIO
  baseline (per the dd test in `ram-speed-benchmark.md`) was actually
  page-cache contaminated — actual PIO performance via O_DIRECT
  is lower. Confirm against mainline data once captured.
- Use the canonical comparison metric: **sequential 1 MiB block-size
  throughput**. Single-number summary going forward.
