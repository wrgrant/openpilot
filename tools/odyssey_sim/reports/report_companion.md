# Pulse analysis and drive companion handoff

Implementation is in `tools/odyssey_eps_report.py` and `tools/odyssey_drive_companion.py`, with two unittest files in `tools/tests/`. Work stayed on `odyssey-sim`; no branch switch, commit, push, device connection, or `openpilot/` write was performed. All runtime dependencies added here are standard library and numpy. Existing cereal, opendbc and LogReader provide rlog decoding.

## What the lead runs

From `~/Code/odyssey-pilot`:

```sh
PYTHONPATH=$PWD .venv/bin/python tools/odyssey_drive_companion.py --host comma --out ~/comma-logs --max-minutes 30
```

Add `--queue` to submit deterministic extra probes. Default is notes only. This command is for the lead during the drive; it was **not run against a host** during this implementation.

Offline analysis:

```sh
PYTHONPATH=$PWD .venv/bin/python tools/odyssey_eps_report.py <route-folder> [<another-route-folder>] -o /path/to/pulse.json
```

The report writes JSON and a sibling `.txt`; the CLI prints plain language. Each route is decoded separately, avoiding alignment across boots. The companion pins the first detected route, excludes segments with `rlog.lock`, sorts segment numbers numerically, pulls through a partial file and rename, and reanalyzes the collected route after new segments arrive. A shot spanning two segments becomes available once its end and samples arrive. Restart reuses existing files and reanalyzes them. It keeps the original remote segment directory names inside its local route folder.

Production access is isolated in `Device`: ssh is batch-only with five-second connection timeout and 15-second process timeout; rsync has ten-second I/O timeout and 20-second process timeout. Failures retry on the next poll (default three seconds), bounded by the inactivity or maximum-runtime exit. No restart, arm changes, unpause or arbitrary param writes exist. The writable allowlist is AgentNote, PulseQueue and pause=1. Param updates use same-directory temporary files, fsync and rename.

Queued shots are written only when both the queue file and live outstanding count are empty. At most six are written, with explicit Condition validation. Local `companion-state.json` records issued route/band/condition fingerprints **before** the write, so an ambiguous disconnect cannot cause automatic duplicate submission. The tradeoff is that a failed write may skip a useful suggestion; the lead can inspect the queue and summary before deciding to retry it. Do not run two companions against the same device simultaneously: queue emptiness checking and writing are not a remote compare-and-swap.

Exit reasons are `pass complete`, `fault pattern: <reason>`, `nothing new for three minutes`, or `max minutes reached`. Faults request pause only. An unsuccessful pause remains explicitly pending in the handoff. The final stdout and `companion-summary.json`/`.txt` contain the exit reason, analyzed coverage, exact missing shot names, conclusions and recommendations. Live `state=done` can arrive before the final segment closes; the handoff reports live progress separately from analyzed coverage rather than claiming those samples were pulled. Five equal abort end records trigger a fault; live abort counters provide earlier conservative detection when logs lag. Counter snapshots cannot establish ordering among different reasons arriving within one poll.

## What William sees

The synthetic CLI run prints:

```text
0s synthetic--0 9 clean; 9 of 26 good. Moves 4-5% / Need: pair-4-0.1, pair-8-0.1
link unavailable / read failed: simulated dropped link; retrying
0s synthetic--1 18 clean; 18 of 26 good. Moves 4-5% / Need: pair-8-0.2, train-8-30+
0s synthetic--2 26 clean; 26 of 26 good. Moves 4-5% / Pass complete; review results
Exit: pass complete. Route: synthetic. 3 closed segments analyzed.
```

The panel receives two lines, each at most 34 characters, for example:

```text
18 of 26 good. Moves 4-5%
Need: pair-8-0.2, train-8-30+
```

On a detected fault it receives `Paused: check test data` and the stop reason. Notes update when closed segments are analyzed; their latency includes segment closure and transfer, typically around a minute, not frame-by-frame feedback. The live status is still polled every few seconds.

## Methods and limits

- **Clean shots:** completed end records with at least 100 samples, start/end coverage within 50 ms, no gap over 50 ms, finite wheel/motor/command/output, and the wheel sensor below 1200 raw units and without a sustained 800-unit reading for 150 ms. Aborted and incomplete-coverage end records remain in the report with reasons. Missing lane/yaw measurements are explicitly unavailable and do not erase useful EPS measurements. Six-column legacy callers retain screening compatibility; production decoding includes the sensor.
- **EPS square map:** identify constant requested-command runs, take the settled second half of each, subtract lead-in motor magnitude, fit `slope * max(command_percent - deadzone_percent, 0)` separately for each command sign, first direction and speed band. JSON includes per-level independent-shot bootstrap intervals, fitted dead-zone/slope residual-grid intervals, and positive/negative asymmetry. Motor is unsigned; sign labels are command sign, never a measured signed motor torque. One sample can yield a point fit but not road-repeat confidence. Short plateaus and downstream rate limits can bias the static approximation.
- **Ramp breakaway:** rate-integrated angle is constrained to the measured angle's +/-0.05-degree bin. Incremental movement at the end of each level is compared to the same-direction/band sham 95th-percentile movement, with a 0.02-degree floor. The preceding level and first moving level form a screening bracket. No shams means no bracket. Ramp history, crown and residual motion prevent calling this a settled static-friction threshold. Later queued single-level probes tighten the bracket.
- **Pulse pairs:** rest and reversal are separate windows through their following zero-command response, bounded by the next pulse. Each has signed wheel displacement, peak absolute wheel rate, unsigned motor peak, and first motor/wheel threshold-crossing latency from the command edge. Thresholds use lead-in noise floors. Delay is sample-resolution onset detection, not a fitted physical transport delay.
- **Trains:** wheel displacement and mean unsigned motor over the active train are compared with an interpolated same-sign, same-band square/ramp steady reference at the same average percentage command. Zero-command baseline anchors the reference. Motor delivery ratio is reported directly (0.5 means half the reference). Sparse steady levels, nonlinear EPS, and short ramp dwell limit this interpolation; no reference bracket gives an explicit unavailable result.
- **Chirps:** short 1.5-second sweeps cannot independently resolve nonparametric low-frequency gain and phase. A second-order ARX input/output fit searches 0-200 ms transport delay, fits the first 80% and reports held-out one-step prediction R-squared. JSON gives gain and phase at 0.5, 1, 1.5, 2, 2.5 and 3 times per second, for command to wheel rate and wheel angle to yaw. These are conditional estimates under a second-order linear assumption, not validation of a nonlinear simulator. Coherence uses pooled Fourier cross-spectra across independent same-height/band sweeps; it is null with only one repeat. Two repeats are still weak evidence, and no confidence interval is claimed for the transfer fit.
- **Signal timing:** yaw is explicitly `carControl.angularVelocity[2]`, the pose-filter source, **not raw gyro**. The JSON states its approximate extra 75 ms delay relative to raw gyro and that no compensation was applied. Angle-to-yaw uses logged entry speed to convert speed*yaw back to yaw, justified only by the test's bounded speed change. The simulator must model this measurement delay or use later raw-gyro analysis. No undocumented device-frame gyro rotation is assumed.
- **Driver push and lane movement:** every measured shot reports peak `75 * bandpass(speed*yaw, 0.3-3 Hz)` in newtons and significant direction changes per second. FFT filtering is zero-padded and de-meaned; sub-three-second shots have weak 0.3-Hz resolution and edge sensitivity. Lane motion is maximum model-derived offset change during the shot, not ground-truth lane displacement. Missing sources remain null.
- **Missing evidence:** coverage is listed against the actual imported `session()` per observed pass and speed band. Reports distinguish clean-shot coverage from remaining uncertainty, including sham coverage, breakaway brackets, steady references and missing signals. A completed pass does not mean a calibrated plant.
- **Suggestions:** deterministic midpoint single-level probes halve broad breakaway brackets; disagreeing mirror pulse responses cause repeats; pulse widths between moving/still results cause midpoint-width pairs. Each explanation is separate from its six-field queue Condition. Suggestions obey 12% maximum height and total lead/signal/tail shorter than `HARD_SHOT_S`. Empty evidence produces no speculative queued probes; narrow brackets and consistent pulse evidence produce none. The existing plan supplies missing base shots.

## End-state checks and evidence

Evidence directory (all generated data are outside the repository):

`/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/companion/`

1. **MET — known-answer synthetic signals.** `test_square_map` recovers dead zone 2.8% and slope 1.12 motor units per percentage point. `test_all_shot_kinds_and_suggestions` recovers ramp bracket [4%, 5%], reversal response less than first-pulse response, and 50% motor delivery for the under-delivering 6%/60%-on train. `test_chirp_known_second_order` checks every frequency against an independently specified discrete second-order transfer with 15% gain and 15-degree phase tolerances, plus the angle/yaw link. `known-answers.json` records numerical outputs.
2. **MET — note/suggest.** Tests check two-line/34-character bounds, deterministic ordering, queue validation, midpoint threshold and width probes, mirror repeats, limit handling, and no suggestions with no uncertain measured result or narrowed consistent evidence.
3. **MET — local end-to-end companion.** `TestRecordedCompanion` writes actual compressed cereal rlogs with CAN motor messages and runs the production decoder/analysis. Three segments yield 9, 18 and 26 clean shots; snapshot 2 drops and recovers; all segments are pulled and exit is `pass complete`. A second drive with five frame-gap aborts exits `fault pattern: frame gap` and writes pause=1. Tests also cover deadlines, param allowlisting, outstanding queue protection and restart deduplication. Separate CLI runs are preserved in `pass.stdout`, `fault.stdout`, `pass-output/` and `fault-output/`.
4. **MET — November 9 drives.** Ran the analysis on all three actual folders `04b44e4c4fe44232_000001ac--114fc8ccff`, `04b44e4c4fe44232_000001ad--fca502745a` and `04b44e4c4fe44232_000001ae--e34de72a0e` under `~/comma-logs`. Exit 0; zero clean/recorded pulse shots; explicit `No pulse shots in these logs.` `nov9.json`, `nov9.txt`, `nov9.stdout` preserve the result. This was local log reading only, not a new evaluation of how the drive felt.
5. **NOT FULLY MET — tests and touched-file lint pass; repository-wide lint fails elsewhere.** `.venv/bin/python tools/test_runner.py tools/tests/test_odyssey_eps_report.py tools/tests/test_odyssey_drive_companion.py -j 1` passes all 14 tests. `.venv/bin/ruff check` with the four changed Python paths says `All checks passed!`. Bare `.venv/bin/ruff check` reports 30 errors in untouched `tinygrad_repo/test/amd/hw/` and `tinygrad_repo/test/mockgpu/amd/` files. Those files are outside the allowed write set and were not changed. Exact outputs: `tests.stdout`, `ruff-touched.stdout`, `ruff-repository.stdout`. The next step for a repository-wide green check is for the lead to resolve or exclude that unrelated tinygrad baseline separately.

`git diff --check` is clean for this worker's files. No device-side behavior, authenticated transfer, actual physical response or panel rendering was tested; local read-back and validation cover the companion interface. There is no `ARCHITECTURE.md` in this checkout; the brief's explicit file scope was retained, and this report supplies the module/dependency handoff instead of creating unrelated onboarding files.

## Text for ODYSSEY.md

Replace item 13's old sentence saying per-kind analysis is still to be written with:

> `tools/odyssey_eps_report.py <route dirs> -o pulse.json` writes JSON and text with the square-wave EPS map and sign asymmetry, sham-screened ramp breakaway bracket, separate rest/reversal pulse response and onset delays, PWM motor delivery against interpolated steady commands, and conditional second-order chirp transfer estimates with repeat coherence. It reports 75 kg driver push in newtons at 0.3-3 times per second, wheel-sensor peaks, lane-offset movement and exact missing shots. Yaw is the pose-filter signal with its extra approximately 75 ms delay explicitly recorded; it is not raw gyro. A single pass is screening, and short chirps do not settle the plant. `tools/odyssey_drive_companion.py --host comma --out ~/comma-logs [--queue] [--max-minutes N]` follows closed segments, posts a two-line note, optionally queues at most six bounded extra probes, and exits with a stand-alone handoff at a completed pass, fault, three-minute inactivity or deadline. It can pause on faults and never unpauses or changes the arm. Test it with `--local <recorded-route-folder>` before a drive.

In “After every drive”, consolidate the force convention to **75 kg**, matching William and this analysis; do not retain a second conflicting 80 kg rule. This report does not edit ODYSSEY.md or any lead-owned file.
