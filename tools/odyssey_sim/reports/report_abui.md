# Driver-controlled Odyssey A/B testing

Implemented in the existing `odyssey-sim` checkout. No branch switch, commit, push, device access or van access. The concurrent `tools/odyssey_sim/shake/` work and pre-existing dirty `msgq_repo` were left alone.

## What William sees and does

With the van off, open **Vehicle → Honda → Odyssey A/B experiment**, choose lane, bump, nnlc_model or lowspeed, and enable **Driver selects A/B arm**. None hides the panel. Reset totals asks for confirmation. The panel sits on the left below the speed header, leaving the centre road area, speed, right developer column and bottom alert area available.

Onroad, tap the large panel to advance one arm. It displays the **controls-acknowledged arm**, rather than immediately pretending the requested arm has taken effect. After switching it shows “5 s after switch”; when eligible it says “counting”. Each arm has separate straight and curve progress toward 300 s and 60 s. An asterisk marks the arm with the least completed target fraction. Hold the panel to confirm a reset. Clock mode displays the arm and progress but ignores taps.

These are actual raylib widgets rendered at 2160×1080 by the new test under `openpilot/selfdrive/ui/tests/`, using synthetic messages and a neutral background. They are not screenshots from the van or a camera replay.

### No experiment: panel absent

![No experiment](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/abui/none.png)

### Counting, torque arc and developer rows

![Counting](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/abui/counting.png)

### Not counting: blinker

![Excluded second](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/abui/blinker.png)

### One arm at both targets

![Target reached](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/abui/target.png)

### Torque bar visible with steering active

![Torque bar](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/abui/torque.png)

The large-layout HUD already calls `TorqueBar(scale=3, always=True, fixed_height=20, center_dot=False, meter=True)`. Its fade is drawn before the HUD, so it does not cover the bar. No production torque-bar or developer-row rendering fix was required. With `DevUIInfo=3`, the harness visibly renders CURVE SPD 46 mph and STEER TRQ 75%, the right column and bottom strip. The existing developer-element tests were plain classes that unittest did not discover; they now inherit `unittest.TestCase`, and all eight run.

## Activation and compatibility

The new persistent keys are `OdysseyABExperiment` (empty by default), `OdysseyABManual` (false), `OdysseyABArm`, `OdysseyABTotals` (JSON string), `OdysseyABReset`, and `OdysseyABStatus`. The params library was rebuilt successfully with the brief's scons command. Installation on the comma must rebuild its params library too.

For a local configuration or the lead's later install, set:

```json
{"OdysseyABExperiment":"lane","OdysseyABManual":true,"OdysseyABArm":"off"}
```

The UI experiment list is lane off/on, bump off/kick/bump, nnlc_model stock/S3, and lowspeed base/firm. NNLC model comparison needs `NeuralNetworkLateralControl` enabled and the model available. Low-speed comparison needs `SteerLowSpeedAngle` enabled; the UI describes these prerequisites. Other features use their always-on settings during a manual test, including suppression of legacy NNLC and wander clocks. NNLC switches reset PID/shaper state. Invalid selections use the safe first arm. An unavailable candidate model is acknowledged and counted as stock.

With the experiment empty, the prior parameter-controlled clocks and behaviour remain available. The new panel's clock mode uses the same 60/180 s block durations and the new safe experiment list; it never selects S5. The historical three-model constants remain for interpreting older logs and preserving the legacy `NnlcModelAB` path. Keep that legacy flag off; use the new selector for stock/S3. This explicitly resolves the brief's tension between preserving old clock behaviour and retiring S5 from the new experiment list.

## Implementation decisions

`odyssey_ab.py` owns the experiment list, targets, pure `good_data`, validated totals and log parser. `LiveAB.update` evaluates frames and uses a bounded non-waiting queue. A worker reads params, emits `cloudlog.event("odyssey_ab", ...)` on changes and each second, and persists totals. Arm selection reads an atomic in-memory tuple. There are no new params reads, filesystem writes or logging calls in the 100 Hz hook. Lost queue entries and gaps between frames receive no credit. Totals save every second and on experiment deactivation; an abrupt process/power loss can lose the last unsaved fraction of a second.

Each log event contains experiment, effective arm, manual, good, reason, straight/curve kind and the sampled monotonic timestamp. The parser handles the real `SwagFormatter` `msg` envelope; the round-trip test exercises it. A deactivation record prevents carrying an old active experiment indefinitely. No capnp schema changed. Offline scoring imports the same eligibility function and uses recorded decisions when available; older logs reconstruct switch and press history with clock fallback. The takeover report extracts fresh rather than accepting a stale cache without records, uses the shared low-speed eligibility and treats presses of at least 1 s as real overrides. Shake diagnostics still include wheel-press time so shaking arms cannot hide behind poor lane-position eligibility.

## Seven end-state checks

| # | Result | Evidence |
|---|---|---|
| 1 | Met | Selection, clock/manual isolation and bad-name fallback; every eligibility reason and straight/curve boundary; malformed totals, JSON persistence format, live worker persistence/reset; actual log formatter round trip into `logged_arm`. Tests: `test_odyssey_ab_ui.py`, `test_odyssey_drive_map.py`. |
| 2 | Met | Real `controlsd_check.py --all-manual` on **1ac segment 5**, all nine safe experiment/arm combinations: **6,000 / 6,000 frames each**, 100% steering active, finite commands, torque controller throughout. [Replay evidence](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/abui/abui_replay_done.log). The check now accepts `--all-manual` and forced params. This minute is mainly above 25 mph; it proves process integration, not low-speed tuning quality. |
| 3 | Met | Five real-widget screenshots above, generated by `test_odyssey_ab_panel.py`; panel absent/counting/excluded/target/torque states. Speed, torque arc and `DevUIInfo=3` rows visibly render in the large layout. |
| 4 | Met | The render test invokes the widget's release handler, observes the arm param change, starts the real params worker and confirms `LaneCentering.active_arm` sees it within a 0.5 s deadline. This is a synthetic tap, not a hardware touchscreen test. |
| 5 | Met on host | 100,000 update calls: mean **1.23 µs**, p95 **1.50 µs**, p99 **1.67 µs**, max **138.42 µs**. Queue-full path also exercised. No Params handle exists on the benchmark update object, proving it needs no I/O. [Benchmark source](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/abui/benchmark.py), [output](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/abui/benchmark.log). Hardware scheduling/latency on the 3X remains a van-only check. |
| 6 | Met, limited evidence | [9 November map](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/abui/drive_1109.html): **2,641 mapped seconds** from 1ac/1ad/1ae, four switchable colourings, popups with exclusions and per-arm table. GPS comes from `gpsLocation`; acceleration is speed × logged yaw rate, filtered 0.3–3 Hz, push is 80 × 95th percentile absolute acceleration over five seconds. Road-matched null result and limits below. |
| 7 | Met | **61 tests passed** through `.venv/bin/python tools/test_runner.py`, including every modified/new test file plus relevant controller regressions. Ruff clean across all changed Python and new files; `git diff --check` clean. [Tests](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/abui/abui_tests_done.log), [ruff](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/abui/abui_ruff_done.log). |

The full test set is the A/B unit tests, map tests, wander/takeover tests, lane/deadband/low-speed/OdysseySteer regressions, panel render/tap test, and developer-element tests. No pytest or new dependency was introduced.

## Map and road-matched comparison

```sh
PYTHONPATH=$PWD .venv/bin/python tools/odyssey_drive_map.py <1ac-route-dir> <1ad-route-dir> <1ae-route-dir> -o map.html
PYTHONPATH=$PWD .venv/bin/python tools/odyssey_wander.py <route-dirs...> --ab lane --road-matched --force
```

The output HTML contains its data and code; Leaflet is loaded from cdnjs and OSM tiles require a network connection. The speed colours are <18, 18–25, 25–45 and ≥45 mph. The table counts mapped one-second samples; live persisted totals are integrated at controls frequency, so boundary fractions and unmapped GPS gaps can differ. GPS fixes are held until the next fix. Push figures in excluded/hand-driven sections describe motion, not controller performance.

Road matching uses approximately 50 m cells, 30° heading bins, and 5 mph speed bands. It pairs per-road arm means and bootstraps **drive-pair means**, rather than pretending nearby GPS samples are independent. The deterministic synthetic test recovers a known +2 N difference and a zero-difference null.

The existing-drive null uses routes 174, 18c, 19b, 1a3, 1a7, 19c, 1aa and 1ab from BRIEF_COMMON: **22 matched cells, four drive pairs**, on−off **+3.60 N**, bootstrap interval **[−0.49, +7.99] N**. It includes zero. [Null evidence](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/abui/abui_null_matched.log).

An earlier position/heading-only estimator treated adjacent cells as independent and falsely excluded zero (+2.89 N, [1.62, 4.23]); that failed null caused the speed matching and drive-pair resampling change. Four drive pairs remain weak evidence; the interval is descriptive and does not prove a treatment effect or fully account for drive pairs sharing a route. On the three 9 November drives, **no qualifying same-road/same-heading/same-speed cells pair different model arms**, so the map reports no paired model estimate rather than inventing one. The driver-controlled repeat passes are needed to supply that evidence.

## Self-review and remaining on-van verification

I reviewed the control defaults/switching, logging/offline interpretation, and UI/test evidence. Fixes from those passes include typed Params API usage, capnp enum comparison, actual cloudlog envelope parsing, stale/deactivation status, always-on NNLC restoration, model fallback acknowledgement, skipped unittest classes, avoiding credit across frame gaps and conservative null-test intervals. I rejected adding schemas, another IPC service, or a dependency: the existing params/log carrier suffices. As an implementation worker I did not delegate further.

Only William and the lead can verify the installed screen's physical legibility/tap response, alert and navigation combinations in actual driving, correct model/low-speed prerequisites on the device, 3X scheduling cost, persistence across ignition/power cycles, and whether manual repeated road sections actually feel better. The harness uses a neutral background and does not exercise camera, alerts or all navigation widths. No claim of road safety, better tracking or reduced takeovers follows from the open-loop replay. Next step: lead reviews this diff, rebuilds/installs with the van off, then William checks the panel and alternates arms over repeat stretches while reporting what he feels.

## Text to add to ODYSSEY.md

> Driver-controlled A/B tests: Vehicle → Honda → Odyssey A/B experiment selects lane (off/on), bump (off/kick/bump), nnlc_model (stock/S3) or lowspeed (base/firm). Enable Driver selects A/B arm and tap the large left onroad panel to advance arms; hold to confirm a totals reset. NNLC and low-speed tests require their underlying feature enabled. Empty OdysseyABExperiment hides the panel and retains existing behaviour. The new clock selector keeps 60/180 s block lengths and uses the safe new arm list; S5 is excluded. Keep legacy NnlcModelAB off. Manual mode suppresses other experiment clocks and preserves their always-on settings. OdysseyABTotals holds validated per-experiment/per-arm straight and curve seconds across drives, targeting 300/60 seconds per arm. The shared odyssey_ab.good_data gate requires steering active, appropriate speed, no blinker/lane change, five seconds after a switch, no sustained ≥1 s wheel press, and visible inner lane lines for lane-position experiments. odyssey_ab logMessage events record the effective arm and live counting decision; the wander and takeover scorers prefer these records and reconstruct clock arms for older drives. tools/odyssey_drive_map.py <route dirs> -o map.html shows arm, push on an 80 kg person in newtons, speed band and counting exclusions. tools/odyssey_wander.py <routes> --ab lane --road-matched --force reports GPS/heading/speed-matched push differences and descriptive drive-pair bootstrap intervals; use repeated passes and do not treat sparse pairs as a settled result. Before installing controls changes, controlsd_check.py <segment> '{}' --all-manual checks every safe arm through the real process.
