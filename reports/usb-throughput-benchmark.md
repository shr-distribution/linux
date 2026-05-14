# HP TouchPad USB Throughput Benchmark

Comparative USB 2.0 high-speed bulk-transfer throughput between legacy
webOS 2.6.35 and mainline Linux 6.18 on the HP TouchPad, captured to
confirm whether mainline has a USB-side regression.

## Hardware

- **Device:** HP TouchPad (Topaz WiFi, 32 GB)
- **SoC:** Qualcomm APQ8060 (dual-core Scorpion @ 1.5 GHz)
- **USB controller:** ChipIdea HSUSB OTG @ 0x12500000, EHCI host with
  TI ULPI PHY
- **USB speed:** 2.0 high-speed (480 Mbps line rate; ~60 MB/s practical
  bulk-transfer ceiling)
- **Connection:** USB cable to host PC, gadget mode (CDC-ECM) for
  TCP/IP networking

## Test Methodology

A single 200 MB random-data payload is served from a `python3 -m
http.server` instance bound to the host's USB-gadget IP. The device
fetches the payload via `wget` from its `usb0` interface to `/tmp`
(tmpfs on both webOS and LuneOS so eMMC write speed doesn't pollute
the measurement). The wget command is repeated 10 times in sequence;
each iteration's wall time is measured by reading
`/proc/uptime` before and after the call.

Throughput in MB/s = `200 / elapsed_s`.

This measures the end-to-end pull path: host kernel → CDC-ECM packet
encapsulation → USB bulk OUT → device USB controller → CDC-ECM
de-encapsulation → device TCP stack → wget → tmpfs write. The
same payload is re-fetched, so the host side serves out of its
page cache (~175 MB/s) — the bottleneck is everything between the
host's USB gadget driver and the device's wget process.

### Why not iperf / netcat

Legacy webOS doesn't ship `nc`, `netcat`, `iperf`, `iperf3`, or
`socat`. Cross-compiling an ARM static binary and pushing via
novacom was avoided to keep the test environment minimal and
reproducible. `wget` is available on both kernels and exercises the
same USB bulk-transfer path.

## Results

### Legacy webOS 2.6.35-palm-tenderloin (3-run sanity check, 50 MB)

| Run | Size | Elapsed | Throughput |
|---:|---:|---:|---:|
| 1 | 50 MB | 7.65 s | 6.54 MB/s |
| 2 | 50 MB | 7.48 s | 6.68 MB/s |
| 3 | 50 MB | 7.53 s | 6.64 MB/s |

**Median: 6.64 MB/s.** Tight distribution (±1%), no degradation
across runs.

For cross-reference, novacom-protocol throughput on the same
hardware (separate measurement, not over USB ethernet):

| Path | Throughput |
|---|---:|
| novacom put (host → device) | 4.8 MB/s |
| novacom get (device → host) | 6.6 MB/s |

Notably the **novacom pull and USB-ethernet pull are
essentially identical** at ~6.6 MB/s, suggesting the bottleneck
is the USB bulk endpoint itself rather than the application
protocol stack.

### Mainline Linux 6.18 (10-run load test, 200 MB)

**Kernel:** `6.18.0-luneos-g18de34fdab20`

| Run | Size | Elapsed | Throughput |
|---:|---:|---:|---:|
| 1 | 200 MB | 9.98 s | 20.04 MB/s |
| 2 | 200 MB | 9.74 s | 20.53 MB/s |
| 3 | 200 MB | 8.99 s | 22.25 MB/s |
| 4 | 200 MB | 9.50 s | 21.05 MB/s |
| 5 | 200 MB | 9.03 s | 22.15 MB/s |
| 6 | 200 MB | 9.93 s | 20.14 MB/s |
| 7 | 200 MB | 10.09 s | 19.82 MB/s |
| 8 | 200 MB | 9.87 s | 20.26 MB/s |
| 9 | 200 MB | 8.57 s | 23.34 MB/s |
| 10 | 200 MB | 8.98 s | 22.27 MB/s |

| Stat | Value |
|---|---:|
| Min | 19.82 MB/s |
| Median | 20.53 MB/s |
| **Average** | **21.19 MB/s** |
| Max | 23.34 MB/s |
| Spread | 17.7% |

**Sustained ~21 MB/s across 2 GB of cumulative transfer.** No
thermal throttling, no buffer-exhaustion slowdown, no errors
across the run.

## Comparison

| Metric | Legacy webOS | **Mainline 6.18** | Mainline / legacy |
|---|---:|---:|---:|
| USB ethernet pull (median) | 6.64 MB/s | **20.53 MB/s** | **3.09×** |
| Best case | 6.68 MB/s | 23.34 MB/s | 3.49× |
| Stability | <1% spread | 17.7% spread | (different rounds) |
| Theoretical ceiling utilisation | 11% | **35%** | ~3× |

### Conclusion: no mainline USB regression

Mainline 6.18 is **substantially faster** than legacy webOS on
end-to-end USB bulk transfer (~3× speedup). The earlier perception
of "slow USB in mainline" likely came from one of:

1. **SCP / SSH overhead** — SSH-over-TCP adds significant encrypt /
   decrypt cost on a 1.5 GHz Scorpion; even a 20 MB/s USB pipe can
   look like 4-5 MB/s through SCP.
2. **eMMC write bottleneck** — pre-triple-fix eMMC fell back to
   1-bit mode at ~5 MB/s. Transfers to an eMMC destination were
   bottlenecked by the storage, not the USB.
3. **Earlier kernels** — pre-`g_ether` or pre-CDC-ECM tuning
   patches in mainline may have been slower; this measurement is
   on `g18de34fdab20` which has all current fixes.

## Why both stacks are still well below the USB 2.0 ceiling

The USB 2.0 high-speed practical ceiling is ~60 MB/s with optimal
bulk transfers. Even mainline at 35% utilisation leaves significant
headroom. Likely contributors to the gap:

- **CDC-ECM packet overhead**: each packet carries Ethernet (14 B) +
  IP (20 B) + TCP (20 B) headers, plus USB token / data / handshake
  packets. For 1500-byte MSS payloads this is ~4% protocol overhead,
  but for the bulk transfer the per-packet USB token cost is
  more significant.
- **TCP windowing and ACKs**: small TCP windows on the device side
  cap the in-flight bytes. tmpfs at the receive side serializes the
  socket reads.
- **Single CPU**: wget is a single-threaded user process; the
  CPU-side packet processing is not parallelised across both
  Scorpion cores.

To push closer to 60 MB/s would require either a different protocol
(direct USB bulk transfer, not CDC-ECM Ethernet), TCP window scaling
tuning, or moving the receive side onto a second CPU. None of these
is a priority — 21 MB/s is more than adequate for kernel/userspace
deployment via SCP, debug log streaming, and the typical novacom-
style workflows.

## Notes on test reproducibility

1. **Host IP**: the USB gadget IP on the host changes interface
   name each boot (`enx<mac>`). Configure with the standard
   one-liner from `feedback_usb_iface_setup.md`. The same iface
   was used for both legacy (with secondary 192.168.0.1/24 for
   webOS's `usb0` at 192.168.0.202) and mainline (172.16.42.1/24
   for LuneOS's `usb0` at 172.16.42.2).
2. **Server**: `python3 -m http.server 8000 --bind <host_ip>` from
   a directory containing the payload file. The host serves out of
   page cache after the first iteration; per-run overhead is
   wget command setup (~50 ms) which is negligible against the
   ~10 s data transfer.
3. **Tmpfs write speed**: not a confound — even legacy's 459 MB
   `/media/ram` tmpfs writes at hundreds of MB/s.
4. **Payload entropy**: random data is used to defeat any
   USB-controller-side compression heuristics (none expected on
   ChipIdea HSUSB but defensive). gzip compression of `/dev/zero`
   could otherwise inflate apparent throughput.
