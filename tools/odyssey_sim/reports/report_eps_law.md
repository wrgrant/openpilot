# Odyssey EPS: command source or rate tracker?

**Verdict: B above 15 mph, with strong predictive evidence and moderate mechanistic confidence.** Motor magnitude depends on steering rate opposing command, beyond what a curved command map explains. This does not establish the Clarity's exact R5/R6/P/D implementation in `39990-THR-A020`. Below 15 mph, none of these simple laws generalizes reliably. Whole-law speed fade is provisionally better than command-only fade at crawl speed; the precise schedule remains unidentified.

## Data and identification limits

Decoded every available segment in all 16 `~/comma-logs/04b44e4c4fe44232_*` routes: 2,217,725 carState frames, approximately 6.16 frame-hours at 100 Hz, including inactive/unusable recordings. 563,629 frames pass screening. Training uses routes 174–1aa; 1ab selects transport lag; 1ac, 1ad and 1ae are untouched route holdouts. Five recordings contribute no eligible frames. Full route IDs, counts, coefficients and diagnostics are in `~/comma-logs/odyssey-evidence/eps_law/results.json`; matching NPZs retain decoded observations.

Screen: `latActive`, actual CAN steering-request bit, fresh command/motor/control (<40 ms), output enabled, `!steeringPressed`, |driver sensor|<100, valid command. Active/requested sensor medians are 78–144 raw; 10th percentiles 15–23, with long hand-input tails (99th percentiles 1,898–2,651). The cut is conservative, not proof of absent hands: the wheel's own motion affects this sensor. At cuts 50/200/300, fitted damping remains positive: 2.11–2.47 raw/(°/s) at 15–30 mph, 2.14–2.34 at 30–45, 1.34–1.87 above 45.

Reused `CANParser`, `LogReader`, `eps_report.fine_angle/segment_key` and `eps.fit.TORQUE_BP`; followed takeover sign conventions. Actual `sendcan` command is negated/4096 into carState-angle convention, avoiding requested-command rate-limit error. Crucially, carState's rate uses the coarse STEERING_SENSORS channel: it is zero in 84–89% of eligible November frames. The finer STEER_STATUS rate is zero in only 4–5%; fits use that 0.1°/s signal.

**Motor direction is not established.** DBC motor is unsigned. Its `UNKNOWN_TORQUE_STATE_BIT` is set for 99% of strong manual steering in either direction; it is usually clear during lane keeping in either direction. It is not a usable direction bit. Models predict magnitude directly; no measured motor sign is assigned from command or motion. Undecoded bits elsewhere could still carry direction. Closed-loop command/rate correlation and hand-sensor selection limit causal interpretation.

## Fits and held-out evidence

A: `m=G·min(max(|u|−d,0),h)`. B: `m=|G·sign(u)·min(max(|u|−d,0),h)−Kd·filtered_rate|`. C-static replaces the capped line with fitted interpolation at existing command knots; C-rate adds damping to it. All have independently fitted speed bands. Training searches d=0–6%, h=.55–1, rate τ=0/10/30/100/300 ms. Lags 0/20/40/60/100 ms are compared on 1ab. Training is thinned to 5–20 Hz; holdouts are scored at full logged frequency. No random frame split.

B coefficients (brackets: 95% intervals from 100 resamples of 10-second training clusters, conditional on selected map/filter/lag):

| mph | G, raw/full command | Kd, raw/(°/s) | d | h | lag |
|---|---:|---:|---:|---:|---:|
| <15 | 70.1 [57.9,83.0] | 1.06 [.65,1.88] | 0% | .55 | 20 ms |
| 15–30 | 116.9 [114.0,119.6] | 2.23 [1.82,2.89] | 1% | .55 | 0 |
| 30–45 | 123.3 [121.4,125.4] | 2.29 [2.12,2.43] | 2% | .55 | 0 |
| >45 | 122.5 [121.9,123.2] | 1.43 [1.32,1.56] | 2% | .55 | 0 |

All B bands select τ=100 ms; this is an **effective telemetry filter**, not evidence of a 100 ms firmware tracker. Grid parameters have no defensible statistical intervals; h reaches its lower search boundary. The flexible C-rate map reaches 69–74 raw at full command above 15 mph, avoiding the hard-cap approximation. A's (G,d,h) are (58.7,0,.55), (100.6,0,.65), (110.4,.01,.65), (120.0,.02,.55). A's G intervals, respectively: [49.2,66.8], [96.2,103.6], [108.0,112.2], [119.4,120.8].

Held-out R², listed as A / B / C-static / C-rate:

| mph | 1ac | 1ad | 1ae |
|---|---|---|---|
| <15 | .345/.260/.405/.366 | .173/−6.972/.110/−6.134 | .406/.493/.382/.424 |
| 15–30 | .604/.855/.609/.855 | .002/.611/.175/.688 | .580/.875/.604/.880 |
| 30–45 | .783/.876/.792/.883 | .734/.917/.739/.917 | .763/.879/.772/.888 |
| >45 | .461/.683/.504/.702 | .403/.643/.448/.661 | no coverage |

1ad's 15–30 mph cell has only 70 frames; its <15 cell has 442. Holdouts total ~888 seconds screened. Unfiltered versus 100 ms filtered B gives pooled R² .783→.879, .831→.886, .512→.660 in the three bands above 15 mph.

Residual is observed minus predicted magnitude. At 15–30 mph, turn-in bias falls −7.74→−3.44 raw with B, exit +3.79→+.76; at 30–45, −5.03→−2.16 and +.50→+.32; above 45, −7.33→−4.50 and −.67→+.28. Remaining turn-in bias shows the law is incomplete. Turn-in/exit here means rate with/against command, not independently annotated road-curve phase. JSON includes rate-bin means/counts and sign-specific RMSE.

Residual plots (absolute paths):

- `/Users/williamgrant/comma-logs/odyssey-evidence/eps_law/residual_0-15.svg`
- `/Users/williamgrant/comma-logs/odyssey-evidence/eps_law/residual_15-30.svg`
- `/Users/williamgrant/comma-logs/odyssey-evidence/eps_law/residual_30-45.svg`
- `/Users/williamgrant/comma-logs/odyssey-evidence/eps_law/residual_45-200.svg`

## Small commands and speed

At 1–3% command, held-out median motor for still/turn-in/exit is 0/1/2 raw at 15–30 mph, 0/1/1 at 30–45, and 1/2/3 above 45. Still means |rate|<.5°/s. For moving samples at 4–6°/s above 15 mph, medians are 3 raw with command (115 frames), 5 against (150). Samples are correlated and unmatched in speed/command; this is descriptive. A universal “under 3% gives no motor” claim is unsupported.

For intuition, above 45 mph B predicts a 2% command at filtered 5°/s gives 7.2 raw magnitude versus zero at rest; polarity cannot be verified. The simulator's assumed .05 N·m/raw gauge makes that .36 N·m, **not calibrated wheel torque**. A 5°/s rate subtracts about 7–11 raw from signed command effort above 15 mph.

Direct speed comparison holds the 15–30 law fixed and fits one multiplier per sub-band on training. Whole-law versus command-only fade held-out RMSE: 0–5 mph 6.28/9.47 raw; 5–10 22.74/37.17; 10–15 9.79/9.98. Whole-law multipliers are .255/.540/.805. Both fail at 5–10 mph; changing only damping also fails at crawl. Prefer whole-law fade as a provisional approximation, without claiming the firmware's exact architecture.

## Consequences and exact documentation proposal

Replace the first measured-fact bullet with:

> Hands-off log fits above 15 mph favor a nonlinear command map minus filtered steering-rate damping. Effective damping is about 1.4–2.3 raw motor units per degree/second. A 1–3% command can coincide with nonzero motor torque while the wheel moves; the apparent 2–4% threshold is not a universal motor dead zone. Motor telemetry is unsigned, so braking polarity and the exact internal firmware law remain unverified.

Append to the speed-fade bullet:

> Whole-law scaling predicts crawl-speed logs better than scaling only command effort, but 5–10 mph remains poorly modeled; the fade's internal placement is unconfirmed.

Append to item 24:

> Command-table inversion alone does not compensate the observed rate dependence; do not import Clarity R5/R6 constants without Odyssey-specific validation.

Curve exits: damping plausibly brakes unwind; symmetric rate cancellation could remove useful braking. Weave: model motor effort as motion-dependent before blaming crossings solely on a fixed dead zone. In `shake/`, compare this candidate with the current static motor map, refit mechanical damping to avoid double counting, and require forward wheel prediction before controller tuning.

Best single discriminator: existing `session()` shot `pair-8-0.2` (1.6 seconds including lead/tail). Compare the same 8% magnitude from rest and on reversal while rate persists, using actual CAN command and the zero-command gap. A predicts equal settled motor after lag; B predicts rate-dependent motor, including during zero command. If the wheel stops before reversal, the shot is inconclusive. Square/ramp levels confound command with accumulated motion; a 2.2-second chirp helps phase estimation but contains too few low-frequency cycles for a reliable transfer function. Retain shams and repeat opposite first directions.

Reproduce: `PYTHONPATH=$PWD .venv/bin/python tools/odyssey_eps/eps_law.py --extract`, then the same without `--extract`. Logs only; no vehicle/device access or git operations. Verification: all four EPS-law tests pass under `tools/test_runner.py`; `ruff check tools/odyssey_eps` passes. Self-review checked equations/units, route separation/coverage, and evidence reproducibility.
