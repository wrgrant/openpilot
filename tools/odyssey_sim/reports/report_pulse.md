# Automatic pulse experiment — implementation and verification

## Lead's review (read this first; it supersedes the numbers below where they differ)

The worker's version could not fire. Replaying three recorded drives through its state machine gave zero shots in 59 minutes of eligible steering, for two reasons:

- **A bug:** `md.laneLineProbs[1:3]` slices a capnp list, which raises. With real messages the runtime latched itself off on its first frame; the process replay "passed" because a latched-off test passes lane keeping through.
- **Entry rules no drive meets:** lane keeping's command inside ±3%, the van within 8 cm of centre and raw lane heading under 0.003 rad, all for 3 s. This van holds 4-5% against crown, sits 8-18 cm right of centre, and its lane heading is noisy frame to frame.

Changes (`odyssey_pulse.py`, tests rewritten against the module's constants):

- Zero baseline kept (it is what bump mode sends), but a shot may start while lane keeping's mean command over the settling second is up to 6%; that value is recorded with the shot as `released`.
- Entry: within 0.25 m of centre with at least 0.50 m from the van's side to the nearer line, low-passed heading under 0.003 rad, 1 s settled, lines at 0.7, lane at least 3.2 m wide, next 3 s under 0.25 m/s².
- Shots are as short as the pulse allows (1.15-2.1 s, down from 2.7 s): drift grows with the square of the time lane keeping is away. Trains and shams last 1.0 s.
- New aborts: the van has moved 0.20 m since the shot began; its side is within 0.30 m of a line; the lane narrows. The offset is gated raw and the heading low-passed.
- The test keeps time on the message clock of the service that paces controlsd, so a replay runs it as the road did, and every drive starts paused.

Evidence:

- **Recorded-drive yield** (five drives, 89 eligible minutes, real state machine on logged inputs, open loop): 150 shots started, 124 completed, 1.4 a minute (2.1 on the best highway drive); aborts: narrow lane 9, lines lost 8, curve ahead 6, others 3. Lane keeping was off 3-6% of eligible time. A 112-shot pass is 55-80 minutes of highway.
- **Check 2, now met:** `controlsd_check.py <1ad segment 10> --pulse` on the real controlsd: selected-but-paused output is identical to off over 6,000 frames; running, there are 16 stretches of exactly zero command and all eight pulse heights appear exactly.
- **Check 3 rerun** with the new limits on the fitted plant, 216 cases at the entry limits, ±0.15 m/s² bias, pulse and train at the 15% ceiling, lanes of 3.3 and 3.7 m: largest distance from centre 46 cm, largest movement in a shot 21 cm, closest the van's side came to a line 29 cm. The plant is weak on short pulses; this bounds the logic, not the van.
- **Check 5, now met:** the panel renders in every pulse state (the earlier failure was the Mac's display being asleep).
- 324 controls and tool tests plus the UI tests pass; lint clean.

**Shortened for one trip (William: 80 minutes of highway is not available; 10-15 is).** The 336-shot plan is replaced by a 26-shot session, most useful first, of richer shots: a growing square wave (EPS map), a one-way ramp (where the wheel starts to creep), pairs of opposite pulses (the bump's pulse and its reversal), pulse trains, 0.5-3 Hz chirps (the simulator's frequency response) and shams. On the two good recorded highway drives a whole pass replays in 10 and 13 minutes. The panel gained a top strip that switches test without parking and, for this test, a bottom strip showing the wheel sensor's reading with a tap to ignore it. Hands off on straight highway that sensor would stop 0-4% of 2 s windows by itself (five drives). Worst-case check rerun for every shot: side of the van 30 cm or more from the line, 20 cm of movement at most.

Still only checkable on the van: that the driver-torque thresholds do not trip on the wheel's own motion during a pulse, how well lane keeping re-centres after a shot, and the panel's tap.

---

**Review draft; not ready for installation or a hands-off road test.** The controller, controls hook, panel, persistence and analysis pipeline are implemented, but the exact-command real-process replay audit and final UI verification remain incomplete. The report cannot yet infer a physical PWM correction for `bump_comp`. Do not treat the finite-command replay or fitted-plant numbers as a safety validation.

Work stayed in the requested `odyssey-sim` checkout. No branch switch, worktree, commit, push, device access or van access. The concurrent shake work and dirty msgq submodule were left alone. This report is the explicit deliverable exception to the brief's write-directory list. `ODYSSEY.md` was not edited.

## What William sees and does

After the lead completes the outstanding verification and reviews the safety assumptions, parked settings are **Vehicle → Honda → Odyssey A/B experiment → pulse**. Selecting it starts paused, without an A/B arm. The onroad panel shows waiting, lead-in, pulse, tail, re-centring, pause or done, completed shots / 336 for the current speed band, and the last shot's wheel-angle change or abort reason. Tap to pause/resume. **Pulse shots → Reset** in parked settings confirms a reset of both bands. `Driver selects A/B arm` does not control this experiment.

The host configuration is `OdysseyABExperiment="pulse"`, `OdysseyPulsePaused=true` initially. Progress is `OdysseyPulseProgress`, with a version and completed condition ids per band; `OdysseyPulseReset` requests a reset. Params library rebuild completed with the brief's scons command. A device install must rebuild its library too. The hook is restricted to the Nidec `HONDA_ODYSSEY` fingerprint and torque controller; other vehicles cannot run it.

The test waits for three seconds of eligibility, owns the final torque request for 2.7 seconds, then allows lane keeping to recover. An aborted condition remains first in its band's pending plan and is retried once settled. Both speed bands finish independently and do not automatically reset. A process/power loss can repeat a shot whose progress was not saved. It never intentionally marks an aborted shot complete.

## Command and plan

`odyssey_pulse.py` is the pure state machine plus worker adapter. The hook follows `OdysseySteer.apply`, replacing the final command rather than adding anything. The lateral controller continues running and is reset every shot frame, preventing integral accumulation while its output is discarded. Normal completion returns toward the live lane-keeping command by at most 0.01 normalized torque per frame, including after the last condition. Safety aborts immediately return the live normal command; the existing Honda rate limiter and safety controller still mediate CAN output. A fault in the runtime or its outer hook falls back to normal lane keeping. Internal faults latch the test off until process restart.

The baseline is **zero**, logged on every shot. The project measurements show a crown-holding mean of 0.04–0.05; a frozen mean plus the requested 0.12 pulse would reach 0.16–0.17 and violate the 0.15 ceiling. Zero avoids silently clipping a condition or mixing inside/outside-deadband baselines. The conservative entry gate requires the normal request within ±0.03 for the full settling interval. This may give very low shot yield on crowned roads. Zero baseline removes crown compensation for the entire shot: shams and drift aborts are essential, and shams cannot prove that crown was constant between different shots. A future nonzero-baseline experiment needs its own labelled conditions and amplitude budget.

Each shot is 0.5 s quiet lead-in, the condition, and quiet tail to 2.7 s total (at least 0.7 s tail). Outside the pulse every request is exactly zero. Conditions:

- Singles: 2, 3, 4, 5, 6, 8, 10, 12%; 0.05, 0.1, 0.2, 0.4 s; both command directions: 64.
- Trains: 5, 6, 8%; duty 20, 40, 60%; period 0.25 or 0.5 s; 1.5 s duration; both directions: 36. Edges use integer 100 Hz ticks.
- Twelve sham conditions per pass, six labelled each direction: 10.7% of shots.
- Deterministically shuffled full passes, seed 1789 plus repeat index, three passes before stopping: **336 shots per band**. No condition repeats before a full pass. A truncated pass is randomized, not guaranteed balanced by every factor in every prefix.
- Bands are 25–45 and 45–70 mph; 45 belongs to the upper band. The formerly unbounded upper band is capped conservatively at 70 mph.

Minimum time is about **32 minutes of continuously eligible driving per band**, 64 minutes total, before extra recovery, pauses or aborts. Three repeats are a screening pass, not sufficient evidence for every setting. The report requires at least 10 complete shots per exact condition / direction / speed band and 10 matched-direction shams before declaring reliable movement. Obtaining that coverage takes at least four completed plans, pooling routes after explicit resets. They should span multiple drives and road sections. The minimum count is a reporting guard, not a power calculation or statistical guarantee.

The last meaningful wheel-rate sign (>0.05°/s) before entry is recorded, so continue/reverse populations can be inspected. The current summary reports their counts; it does **not** yet estimate separate reversal-conditioned thresholds.

## Limits and assumptions

| Gate | Value | Purpose / limitation |
|---|---|---|
| Absolute request | ≤0.15 maximum | Final test request, not a guarantee about actual motor torque. Plan maximum is 0.12; plant stress uses 0.15. |
| Entry centre / abort | ±0.08 m / ±0.25 m | Start close, abort before ordinary lane clearance is consumed. Width of van/lane and model error still matter. |
| Entry heading / abort | ±0.003 / ±0.010 rad | Bound initial lateral velocity and growing drift. |
| Straight ahead | ≤0.15 m/s² planned acceleration | Whole available 0–3 s model yaw-rate × speed horizon plus current action curvature × speed²; incomplete/nonfinite horizon rejects entry. This is a predicted path, not a guaranteed road geometry measurement. |
| Speed | 25–70 mph | No low-speed EPS regime, no unlimited high-speed extrapolation. During shot ≤0.5 m/s from entry and same band; settling ≤0.3 m/s from its anchor. |
| Lane confidence | Both inner lines ≥0.8 | Stronger than ordinary lane correction. Existing lane pose fit uses first 25 m; confidence is not calibrated positional accuracy. |
| Driver torque | ≥800 raw Honda units for ≥0.15 s, or ≥1200 immediately | Avoid raw `steeringPressed`, which wheel motion triggers. These thresholds are **provisional**, above the measured 400–600 manual-assist dead zone and below/at the common Honda press threshold; they have not been validated against pulse-induced sensor motion or gentle real takeovers. |
| Wheel settling | ≤0.5°/s, torque <800 | Reduces entry while the wheel is already moving. |
| Blinker, brake, lane change, steering inactive | Immediate abort | Any one returns normal steering on the current frame. |
| Model age / frame gap | ≤0.15 / ≤0.03 s | Stale input, nonfinite input and scheduling gaps abort. Actual takeover after a stalled process necessarily waits until the next executing frame. |
| Shot timeout | 2.8 s | Independent hard ceiling beyond the planned 2.7 s. |
| Worker heartbeat | ≤0.5 s | A logging/params worker failure or queue overflow disables experimental output. |

Safety assumptions requiring review: model lane centre and heading are geometrically correct; confidence detects lane loss; the 3 s model horizon predicts approaching curves; ordinary lane keeping can recover from the entry conditions and abort envelope; road width and adjacent traffic leave sufficient clearance; driver attention and ability to override remain available; stock EPS firmware, panda limits and Honda rate limits remain unchanged; CAN output and timestamps are valid; no other process alters actuators after this hook except the known carcontroller/safety path. Sensor torque cannot establish that hands are off or detect every gentle takeover. Wind, crown changes, model bias, lane merges, road edges, construction, obstacles and unexpected traffic are not independently sensed by this test. A speed/heading/offset limit is not a collision avoidance mechanism.

No road-safety claim follows from this implementation. A fault disables experimental ownership, but a process stall, failure of normal lane keeping, or incorrect perception remains a system-level hazard. Existing longitudinal control and driver override behavior remain outside this experiment.

## Record and analysis

Worker-side `cloudlog.event("odyssey_pulse", ...)` emits entry, every shot-frame command with observed `carOutput`, and completion/abort records. The record includes condition id, full condition, zero baseline, monotonic start/end, entry speed/band, last wheel-motion sign and completion/abort reason. Frame records distinguish the controlsd request from delayed/rate-limited `carOutput`; no assertion that CAN follows square edges is made. Queue operations are non-waiting. All params reads, writes and cloudlog calls occur on the worker, including persistence after completed/aborted shots. Logging remains subject to the existing cloudlog transport dropping records, process death and power loss; the report excludes shots without completed records and full sample coverage.

Run on host:

```sh
PYTHONPATH=$PWD .venv/bin/python tools/odyssey_eps_report.py <route-dir> [<route-dir> ...] -o <output.json>
```

The tool reads actual `logMessage` cloudlog envelopes, CAN motor magnitude / wheel angle / wheel rate, `carOutput` and `carControl.angularVelocity`. It aligns each route separately to avoid mixing monotonic clocks from different boots. The fine wheel angle integrates measured rate with actual sample spacing, bounded within the CAN angle's ±0.05° quantization bin. Van acceleration is speed × yaw rate; VSA lateral acceleration is not used. Per-shot force is 75 kg × peak absolute acceleration change from the lead-in mean. Direction changes ignore acceleration changes below 0.01 m/s²; they are raw within-shot changes, not the earlier drive report's band-pass metric. The motor is unsigned: no signed motor-torque measurement is invented.

JSON and stdout include per-setting wheel response and 95% bootstrap interval against direction/band-matched shams, peak wheel rate, motor response, observed average command, acceleration, force, reversal rate, conservative observed-maximum <7 N flag, sample counts, threshold grid brackets and nominal-vs-delivered train command ratio. Intervals resample shots, not drives; effects shared within a drive can make them too narrow. Aborted shots are excluded from response inference, which can bias results toward easy conditions; do not interpret the <7 N flag as an unconditional comfort guarantee. Missing/aborted conditions do not establish a nonresponse threshold.

**Incomplete physical inference:** `rate_limit_corrected_on_time_s` corrects the measured command-area loss only. `corrected_on_time` for physical EPS/bump compensation is deliberately null. Unsigned motor magnitude and pulse-only measurements do not identify equivalent steady torque through nonlinear friction, especially reversals. The exact next analysis step is to add a reviewed steady-torque reference protocol within the same limits, compare its motor and wheel response to each train, and fit separate speed/direction/reversal curves before recommending `bump_comp` on-time. No controller tuning is changed by the report.

## Seven end-state checks

Evidence files live in the brief's [pulse scratch folder](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/pulse).

| # | Status | Evidence and exact remaining work |
|---|---|---|
| 1 | Met at unit boundary | Nine `TestPulse` methods exercise wait/lead/pulse/tail/recover/done handover, retry, every named safety abort, timeout, pause/deselect, nonfinite data, sustained driver torque, internal fault, queue overflow, command phases/limits, train tick counts, full-pass ordering, progress round trip and stale-version rejection. Final [tests.log](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/pulse/tests.log). Worker reset/power-loss durability is not proven by codec tests. |
| 2 | **Not met** | Real `controlsd_check.py` on **1ac segment 5**, pulse selected/unpaused: **6,000/6,000 frames**, 100% lateral active, finite commands, torque controller throughout; [replay_final.log](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/pulse/replay_final.log). This does not prove shots occurred, compare actual shot commands to logged plans, or compare pulse-off output bit-for-bit with pre-change code. The existing checker is unchanged and cannot make those assertions. Next: instrument replay telemetry capture, choose an eligible straight minute, assert nonzero completed-shot coverage and exact request equality per phase, then replay pre-change and current pulse-off code with identical params and compare complete arrays. Do not relax safety gates merely to obtain replay coverage. |
| 3 | Met as bounded plant stress, limited | Feedback uses simulated centre/heading to abort. 36 cases at 11.3/20/31 m/s, both directions, bias −0.15/0/+0.15 m/s², single and train at ceiling 0.15, zero baseline, entry offset 0.079 m and heading 0.0029 rad. Largest absolute drift **0.25228 m**, including one integration frame at abort. [plant.json](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/pulse/plant.json), [checks.py](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/pulse/checks.py). Uses existing plant, explicit CAN/controlsd sign inversion, point-mass lateral integration and ideal pose sensing; stops at abort/tail, so recovery after abort is not validated. Plant has rejected lane-wander realism, weak short-pulse friction/column dynamics, no perception/driver/traffic/lane-edge errors or safety-system faults. Does not prove onroad lane containment. |
| 4 | Met on host | 100,000 whole runtime updates, model fit every fifth frame, synthetic frame/event queue drained outside timing: mean **13.38 µs**, p95 **36.08 µs**, p99 **41.83 µs**, max **881.46 µs**. [benchmark.json](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/pulse/benchmark.json). Constructor warms numpy fit before controls starts. The boundary has no blocking I/O; queue locking and OS scheduling still cost time. Comma 3X timing remains unmeasured. |
| 5 | **Not met on final revision** | Earlier real-raylib harness invocation successfully produced seven state PNGs below and passed, and recover was visually inspected. Later independent UI calls fail before test execution with `GLFW: Failed to find selected monitor`, native exit **139**. Final wording/settings assertions therefore lack a successful final render run. [ui.log](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/pulse/ui.log). Next: rerun the one UI test alone with a working host display, inspect all final PNGs and verify the pause/settings assertions. |
| 6 | Met | Synthetic logMessage envelopes and 100 Hz samples define no wheel motion below 4%, 0.2° above it, 12 shots per setting/direction plus shams. The actual extraction and summary recover **0.04** for both directions, bracket **[0.03, 0.04]**, and no threshold with no evidence. `TestPulseReport.test_known_threshold_from_log_records_and_samples` in final tests.log. Does not validate real CAN calibration or physical train inference. |
| 7 | **Not met** | **47 non-UI tests pass** through the required unittest runner. Ruff is clean across all modified/new Python; `git diff --check` clean; params library rebuilt. Final UI invocation fails at the native display layer as described in #5, so the full requested suite is not green. No claim that every touched integration has an independent behavioral test: controlsd is exercised by replay, Honda settings assertions are in the presently blocked UI call, and retired daemon paths were removed. |

### Earlier harness screenshots

These are synthetic-state actual-widget renders, not van screenshots, and predate the final wording/settings assertion changes.

![Waiting](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/pulse/pulse_wait.png)
![Lead-in](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/pulse/pulse_lead-in.png)
![Pulse](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/pulse/pulse_pulse.png)
![Tail](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/pulse/pulse_tail.png)
![Recover](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/pulse/pulse_recover.png)
![Paused](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/pulse/pulse_paused.png)
![Done](/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/pulse/pulse_done.png)

## Removed and retained

Removed `odyssey_suite.py` and its held-pulse tests; the daemon's torque mode, `hold_correction`, held-progress bookkeeping and torqueOffset publication; controlsd's additive `torqueOffset` hook; the held-pulse report/schedule and its tests. The report is replaced in its existing location, retaining `fine_angle` and adding actual shot-record analysis tests. `LateralManeuverSuite` default is empty. A persisted `odyssey` suite explicitly runs no maneuvers rather than unexpectedly selecting the stock suite. The existing suite/done param registrations remain for compatibility; the done key has no live reader/writer in this daemon. The schema's torqueOffset field remains because schema changes are outside this brief. The stock curvature maneuvers and `follow_lane` support remain. The ordinary Odyssey steering helpers and A/B experiments remain, with their clocks suppressed during pulse selection and their output discarded only during active shots.

Self-review found and corrected the timestamp member integration error, final-condition handover jump, nonfinite clock/fault handling, queue-overflow abort preservation, cross-route monotonic alignment, first-fit latency and non-Odyssey activation. No review subagents were spawned: this was an implementation-worker task. The absence of a repo architecture map/check workflow was not expanded into a separate tooling project or an out-of-scope edit.

## Text for ODYSSEY.md after completion

> Replace the held-pulse characterization item and the torqueOffset code-map step with: The `pulse` experiment runs a 100 Hz controller after OdysseySteer, replacing the final command during 2.7 s zero-baseline shots. Select it parked in Honda settings, tap the panel to resume/pause, and reset progress in Pulse shots. There are 336 shots per speed band (25–45 and 45–70 mph), with randomized full passes of singles, PWM trains and shams. It waits for stable straight/centred conditions, returns immediately to lane keeping on abort, and persists completed conditions. odyssey_pulse records carry condition, baseline, timestamps, outcome and per-frame request/carOutput. tools/odyssey_eps_report.py reads route folders and outputs response/noise, wheel-motion brackets, force on 75 kg and train command-area results as JSON. Three repeats screen conditions; reliable per-setting conclusions require at least ten shots and ten matched shams, plus repeated-road confirmation. Physical PWM/bump compensation needs an additional steady-reference calibration. This experiment requires completed replay/UI and safety review before installation; the current implementation report records outstanding checks.

Only the lead and William can later verify installed UI legibility and taps, physical EPS sign/rate limiting and motor calibration, torque thresholds against hands-off wheel motion and real takeovers, road crown/curve/perception abort behavior, actual re-centring, thermal and scheduling cost on the 3X, power-cycle persistence and human comfort. Those checks must follow completion of the host gaps; this worker did not access either vehicle or device.
