# Upstream Review Status

Companion to [STAGES.md](STAGES.md). STAGES.md is the static roadmap (what
each series does and when it ships); this document is the **live status**
(where each branch sits in send/review right now, and what the most recent
Sashiko/maintainer findings were).

Update **the row** when a branch's state changes. Append new findings to
the **Findings Log** at the bottom. Bump the "Last updated" stamp every
time you change either.

**Last updated:** 2026-06-02 16:15 CEST

---

## Tooling status

| Component | State | Note |
|---|---|---|
| `claude-cli` (Opus 4.7) | **rate-capped** | Burned 5h window on AM Stage-1 chain; Stage-N invalid-arrays + 0 tokens until window resets |
| `claude-cli` (Sonnet 4.6) | **rate-capped** | Same failure mode after burn-down |
| Vertex (Gemini 2.5 Pro) | **working** | After ADC project-id fix to `project-fbffa5c0-2f85-4d78-8be` (16:00) |
| `sashiko-preflight.sh` | working | Reminder: `--repo` points at integration tree, no `third_party/linux` fetch needed |
| `pre-send-check.sh` / `send-series.sh` | working | Use before every `git send-email` |
| Device 172.16.42.2:22 | **g8afabe0a57af running** | gdsc Set A v2 + Set B + cy8c fuzz fix all live and verified |

---

## Stage 1 — clocks / interconnect / wake / thermal

| # | Branch | Commits | Sent | Last Sashiko | Pending |
|---|---|---|---|---|---|
| 0 | `clk-gdsc-msm8x60-legacy` (Set A v2) | 2 | v1 `20260602050840.435933-1` | r3 attempts 2026-06-02 stage-failed (auth) — **re-run needed** | Re-run with Vertex → v2 send |
| 0b | `clk-gdsc-preexisting-fixes` (Set B) | 3 | **ready to send** | 2026-06-02 15:52 Vertex r1 ✅ 4 findings, all `preexisting:true` — confirm Set B fixes (see Findings Log) | Send (keep as 3 patches; 3/3 bundles 2 findings) |
| 1 | `dt-bindings-gcc-msm8660-cleanup` | 3 | v1 `20260602042747.277270-1` | v4 clean earlier | Wait for maintainer review |
| 2 | `clk-mmcc-msm8660` | 3 | v1 `20260602043623.285901-1` | r7 clean (10:40, 596 KB log) | v2 reroll drafted (unhalt -EPROBE_DEFER + GFX2D resets); blocked on #0 v2 |
| 3a | `clk-lcc-msm8960-hardening` | 3 | v1 `20260602045002.290918-1` | v5-r2 clean | Wait for review |
| 3b | `clk-lcc-msm8660` | 2 | held | clean earlier | Unblocked once #1 PLL4_VOTE lands |
| 4 | `interconnect-msm8660` | 2 | not sent | r9/r10 stage-failed today (auth) | Re-run with Vertex → send |
| 5 | `irqchip-msm8660-mpm` | 2 | not sent | v5 clean earlier (MPM rework) | Re-run with Vertex → send |
| 6 | `thermal-pm8901-tm` | 3 | not sent | v4 clean earlier | Re-run with Vertex → send |
| side | `clk-qcom-pll-vote-null-check` | 1 | sent standalone | clean | Wait for review |

## Stage 2 — charger / phy / crypto / media VFE fix

| # | Branch | Commits | Sent | Last Sashiko | Pending |
|---|---|---|---|---|---|
| 7 | `media-camss-vfe-17x-wm-done-fix` | 2 | not sent | v3 ready | Re-run with Vertex → send |
| 8 | `phy-usb-hs-vendor-init-seq` | 2 | not sent | v5 clean earlier (uint8-array schema) | Re-run with Vertex → send |
| 9 | `power-max8903-dc-limit` | 2 | not sent | v5 clean earlier (source_lock + DT gpio_value=0 require) | Re-run with Vertex → send |
| 10 | `crypto-qce-msm8660` | 2 | not sent | v5 ready; r2 (Criticals + 4 High) pending | Re-run with Vertex → send |

## Stage 3 — standalone new drivers

| # | Branch | Commits | Sent | Last Sashiko | Pending |
|---|---|---|---|---|---|
| 11 | `media-mt9m113` | 2 | not sent | Stage 3 r1 done | Review findings (if any) |
| 12 | `input-cy8ctma395` | 2 | not sent | r7 stage-failed today; **fuzz fix live + verified on device** | Re-run with Vertex → send |
| 13 | `power-supply-palm-a6` | 2 | not sent | r1 done | A6 split into 5 patches in progress |
| 14 | `dt-bindings-mfd-lm8502` | 1 | not sent | r1 done | Review |
| 15 | `mfd-lm8502` | 1 | not sent | r1 done | LM8502 r2 confirmation pending |
| 16 | `leds-lm8502` | 1 | not sent | r1 done | Review |
| 17 | `input-lm8502-haptic` | 1 | not sent | r2 (D10 race fix) clean | Review |

---

## In-flight

_(Things actively running right now. Move to a Stage row when done.)_

- _(none)_

---

## Maintainer reply threads sent

| Date | Series | Thread / msg-id | Subject |
|---|---|---|---|
| 2026-06-02 | mmcc-msm8660 | (in reply to `20260602043623.285901-1`) | Confirm Critical mutex_init race fix |
| 2026-06-02 | mmcc-msm8660 | (in reply to `20260602043623.285901-1`) | F9.b fabric-port + GFX2D resets |
| 2026-06-02 | mmcc-msm8660 | (in reply to `20260602043623.285901-1`) | unhalt fabric ports -EPROBE_DEFER cross-check |
| 2026-06-02 | gcc-cleanup | (in reply to `20260602042747.277270-1`) | YAML clock-names third entry |
| 2026-06-02 | gcc-cleanup | (in reply to `20260602042747.277270-1`) | clk-pll vote NULL-check follow-up |

---

## Findings Log

_(Most recent first. Each entry: date / branch / round / model / outcome.
For non-empty findings include the count by severity and a one-line summary
of the most actionable one.)_

### 2026-06-02 15:52 — `clk-gdsc-preexisting-fixes` r1 (Vertex gemini-2.5-pro) — 4 findings, all preexisting:true ✅

4 findings, 3 patches in Set B (PATCH 3/3 fixes two distinct adjacent bugs in `gdsc_unregister` — kept as one commit per user decision):

| # | Finding (preexisting:true) | Loc | Set B fix |
|---|---|---|---|
| 1 | `gdsc_unregister` missing `pm_genpd_remove` → resource leak; pm_genpd_init re-probe would -EEXIST | gdsc.c:653 | PATCH 3/3 |
| 2 | `gdsc_unregister` tears down subdomains **before** of_genpd_del_provider → race window where a consumer attaches to a domain mid-removal (UAF risk) | gdsc.c:649 | PATCH 3/3 (same commit, reorder + add pm_genpd_remove loop) |
| 3 | `gdsc_poll_status` treats negative `gdsc_check_status` errno as boolean true → returns 0 (success) on regmap failure | gdsc.c:109 | PATCH 1/3 |
| 4 | `gdsc_init` ALWAYS_ON branch discards `gdsc_enable` return value → registers genpd as ON when silicon is OFF | gdsc.c:483 | PATCH 2/3 |

All 4 are flagged `preexisting: true` — equivalent to a clean review for a "preexisting fixes" series. review_inline produced a maintainer-style reply asking clarifying "did the original do X?" questions; useful for the cover letter narrative.

324K tokens in / 10K out / 50K cached, 18 min. Full log: `/tmp/sashiko-gdscB-vertex2.log`.

### 2026-06-02 12:30-13:00 — multiple branches (claude-cli, both Opus + Sonnet) — Stage N invalid-arrays + 0 tokens

- gdsc-msm8x60-r3, gdsc-preexisting-r1, mmcc-r8, interconnect-r10, cy8c-r7 all failed with `Stage N failed to produce valid 'concerns' and 'dismissed_concerns' arrays after 3 attempts` and `0/0/0` tokens.
- Cause: 5h claude-cli rate window exhausted earlier in the morning by the Stage 1 confirmation chain. Falsely looked like a sashiko bug for several hours.
- Fix: switched provider to Vertex; auth tripped on ADC project mismatch; corrected to `project-fbffa5c0-2f85-4d78-8be`; first Vertex run (above) succeeded.

### 2026-06-02 10:40 — `clk-mmcc-msm8660` r8 (claude-opus-4-7) — clean

- 596 KB JSON; no concerns. Validated the unhalt fabric -EPROBE_DEFER + GFX2D[01]_AHB_RESET v2 rework.

---

## Next-step backlog

1. **Send Set B** (`clk-gdsc-preexisting-fixes`) to lkml — sashiko confirmed. 3 patches, keep as-is (per 16:15 decision).
2. Re-run Set A (`clk-gdsc-msm8x60-legacy`) on Vertex to confirm the 9 maintainer-finding folds are clean → ship v2 reply.
3. Run Vertex chain on the 6 "ready, never-reviewed-today-with-Vertex" branches: interconnect, mpm, pm8901-tm, qce, max8903, phy → first sashiko clean → send.
4. Address mmcc v2 reroll after Set A ships (cover-letter dep on #0).
5. Continue A6 5-way split (#131).
6. LM8502 r2 confirmation (#128).
