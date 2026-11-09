# Which Odyssey steering roughness can William perceive?

Research and offline metric, 2026-11-09. Implementation: `~/Code/odyssey-pilot/tools/odyssey_sim/perception/metric.py`, branch `odyssey-sim`, uncommitted. Read `ODYSSEY.md` and `BRIEF_COMMON.md`. No device access, push, branch change or edits to another worker's code.

## Recommendation for the NNLC workers

**Stop using torque sign reversals as a smoothness rejection gate. Score the wheel/body outputs after the EPS, and retain hand-torque uncertainty explicitly.** In 55 matched replay-through-plant windows NNLC has more command reversals but lower output roughness. This is a concrete counterexample to the old ranking rule, not proof NNLC is better in real closed loop.

William's report is consistent with mostly small output motion: the revised metric gives zero excess in 85–93% of eligible ten-second windows on 1a7/1aa/1ab. That is a consistency check against an aggregate subjective report, not labeled-event validation. No timestamp identifies his perceived event, and absence of a complaint is not a measured psychophysical trial. Slow unwind is a separate reference-conditioned check; it does not dominate smoothness scoring.

Do not infer that high friction makes every signal vanish. The motor magnitude follows command magnitude well, and the driver's torque sensor retains command-correlated ripple. Wheel displacement is attenuated and high-frequency body response is barely coherent with the command. Torsional reaction can remain even when the wheel sticks. The magnitude of felt hand torque is not identifiable in N·m from these logs.

![Frequency response proxies and matched controller comparison](perception_summary.png)

## Data and identifiable propagation

Reused plant CAN caches for 18c, 19b, 19c, 1a3, manual-only 1a4/1a5, 1a7, 1aa and 1ab. Freshly read every available rlog segment in those routes through `LogReader(file_path)`, extracting `carControl.angularVelocity[2]`, `latActive`, initial params, and `carParamsSP`. New caches: `perception_data/<route>.npz` and `.json`; reproducible extractor: `perception_extract.py` in this scratch directory. Body acceleration is `vEgo * yaw_rate`, not the unusable VSA signal. No roll-based correction is used: this is kinematic lateral acceleration, not the complete lateral gravito-inertial sensation at the seat, which also depends on bank, roll and body location.

Commands are decoded `sendcan STEERING_CONTROL.STEER_TORQUE / 4096` from the existing CAN caches. Motor magnitude is `STEER_MOTOR_TORQUE.MOTOR_TORQUE`; angle and rate are the EPS CAN signals. The primary smoothness metric uses fine angle plus rate. For transfer estimates, integrate rate without the quantized-angle clamp within each analysis block; the clamp itself creates apparent high-frequency angle motion. Driver torque is `STEER_STATUS.STEER_TORQUE_SENSOR` in **raw sensor units**.

Analysis eligibility: speed >=8 m/s, lateral active, no `steeringPressed`, no blinker, no segment/time gap >30 ms. This reduces overt driver interference; it does not prove hands off or hands on. Do not extend this result to parking/low speed. CAN indexing at nominal 100 Hz approximates a uniform grid; the public metric requires uniform arrays. Ten-second metric windows, 5 s hop. Transfer uses independent 20 s eligible blocks, Welch spectra with 10 s subwindows and 0.1 Hz resolution. Pool cross/auto-spectra and compute `H1 = Sxy/Sxx`, coherence `|Sxy|²/(Sxx*Syy)`. No causal closed-loop identification or confidence intervals are claimed.

Amplitude strata use command RMS above 0.5 Hz: small <0.03 (39 blocks), medium 0.03–0.10 (116 blocks). No eligible >0.10 block exists in this selection. Small high-frequency ripple may ride on a large DC command, so this stratification does not directly test the zero-centered motor deadband.

### Frequency results, medium command-ripple stratum

`H1` gains carry output units per normalized command, except motor uses **absolute command** as input. Coherence is median over the band. Fractions of coherent output power are statistical association, not fractions of command physically transmitted.

| Hz | Wheel gain, deg/command | Wheel coherence | Torque-sensor gain, units/command | Sensor coherence | Body gain, m/s²/command | Body coherence | Motor-envelope gain, units/abs(command) | Motor coherence |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.1–0.5 | 7.097 | .677 | 533 | .798 | .876 | .609 | 100.8 | .935 |
| 0.5–1 | 2.505 | .655 | 656 | .721 | .590 | .509 | 93.1 | .907 |
| 1–2 | 1.941 | .651 | 636 | .679 | .744 | .387 | 84.0 | .810 |
| 2–5 | 1.107 | .449 | 780 | .395 | .243 | .008 | 85.0 | .617 |
| 5–10 | .723 | .337 | 1131 | .219 | .074 | .004 | 90.6 | .335 |

Frequency-normalized to the 0.1–0.5 Hz wheel gain, wheel displacement gain falls to about 35%, 27%, 16%, 10%. **These are descriptive response ratios, not causal transmission efficiencies.** In the small-ripple stratum the corresponding wheel gains are 3.961, 1.800, 1.209, .666, .483 deg/command. The amplitude-dependent difference supports treating the EPS as nonlinear rather than imposing one fixed filter on all commands.

For medium ripple, total wheel displacement RMS from rate integration is about the hundredths-of-degree scale at higher frequencies (see JSON for exact spectra-derived RMS); driver sensor RMS is 21–32 units across 1–10 Hz. Above 2 Hz body median coherence is below .01. At 5–10 Hz only about 0.43% of body output power is linearly coherent with the command. Most observed body power there cannot be assigned to command chatter by this analysis.

Motor-envelope gains remain high at 5–10 Hz. The motor is **not** simply absent whenever the wheel is stuck. Because motor CAN torque has no sign, symmetric +/− command chatter can have nearly constant magnitude; it is impossible to reconstruct a unique signed command→motor→wheel→hands chain from this field. Multiplying motor magnitude by command sign would bake the desired answer into the data. This report deliberately does not do so. Separate measured links are the identifiable result.

Likewise, hands-on transmission percentage cannot be computed: no contact sensor or counts-to-N·m calibration exists, `steeringPressed` misses light contact, and the driver sensor measures column torsion/reaction as well as active hand effort. A quiet wheel is not proof of zero haptic torque. The observed sensor gains do not monotonically fall with frequency, which falsifies the simplistic claim that friction suppresses all hand sensations.

The measured ~0.87 s aggregate lag alone does **not** establish low-pass attenuation: a pure delay has unit magnitude response. The observed spectra, deadband, and stick-slip provide the attenuation evidence. Do not turn the aggregate lag directly into an invented first-order filter.

### Rapid actual sign reversals

Found 1,590 eligible one-second windows with >=3 nonzero command sign changes (zero samples skipped). Median / p90 / p99:

| Quantity | Median | p90 | p99 |
|---|---:|---:|---:|
| Command standard deviation | .0469 | .0909 | .1705 |
| Locally detrended fine wheel angle RMS, deg | .0278 | .1714 | .4958 |
| Locally detrended driver sensor RMS, raw units | 48.2 | 106.7 | 283.2 |

The median rapid-reversal window barely moves the wheel. The tail includes larger motion and cannot be declared imperceptible. These windows include road and feedback response and are not isolated reversal impulses. The RMS values are not command-attributable causal gains, and 400–600 raw units of manual-assist dead zone are **not** a human perception threshold. Detailed event rows are in `perception_results.json`.

## Human factors: what can be mapped honestly

1. **Steady steering force and angle discrimination:** Newberry, Griffin & Dowson (2007), *Driver perception of steering feel*, found roughly 15% changes distinguishable in their steady-state force/angle experiments. The evidence supports a relative Weber-like scale, not a universal absolute steering JND in degrees or N·m. Their finding that subjects organize force and angle perception differently also argues against scoring normalized actuator command directly. [Primary paper](https://journals.sagepub.com/doi/abs/10.1243/09544070JAUTO415).

2. **Rotational wheel vibration:** Giacomin et al. (2004) tested 15 people at 18 frequencies, 3–315 Hz, deriving a steering-specific frequency weighting and annoyance results. Equal-sensation weighting and annoyance are different from absolute detection. This supports scoring rim motion by frequency, but neither a single generic hand-vibration weighting nor an arbitrary wheel angular-acceleration limit should be presented as their validated detection threshold. [Primary paper](https://www.sciencedirect.com/science/article/pii/S016981410400006X), [author manuscript](https://bura.brunel.ac.uk/bitstream/2438/1435/1/gsdr2004fwf.pdf).

3. **Grip and steering vibration:** Giacomin & Onesti (1999) tested 25 subjects, 4–125 Hz at two suprathreshold acceleration levels. Frequency strongly affected perceived discomfort; grip differences were statistically significant but generally small in that experiment. This does not justify assuming grip is irrelevant near threshold or when hands barely touch the wheel. [Primary paper](https://hcdstudios.com/wp-content/uploads/2019/08/go1999eof.pdf).

4. **Lateral pulse comfort:** de Winkel et al. (2023), *Standards for passenger comfort in automated vehicles: Acceleration and jerk*, tested 23 participants with varied acceleration/jerk pulses. It discusses earlier lateral comfort thresholds around 0.84 m/s² and .42 m/s³ at 20% discomfort reporting and 1.18 m/s² / .6 m/s³ at 50%. These are comfort results under particular profiles/filtering, **not** universal absolute detection thresholds. The study motivates retaining both acceleration magnitude and time course rather than scoring jerk alone. [Primary paper](https://www.sciencedirect.com/science/article/pii/S0003687022002046).

5. **Translational motion detection:** Soyka et al. (2011) experimentally varied sinusoidal, triangular and trapezoidal acceleration pulses at 1.5, 2.36 and 5.86 s durations. Detection depended on both duration and profile. This supports frequency/profile-sensitive thresholds and prevents equating one peak-jerk value with perceptibility. Their laboratory detection task is not an in-traffic driver roughness rating. [Primary paper](https://link.springer.com/article/10.1007/s00221-010-2523-9).

Mapping: wheel angle/rate can support proprioceptive and rotational-motion proxies; tangential rim acceleration requires assumed wheel radius. Column sensor counts require a measured torque scale to map to N·m and known hand contact to interpret as hand load. Body `v*yaw` is a useful controller-output proxy but not a seat accelerometer or full vestibular signal. The implemented absolute floors below are explicitly **engineering screening assumptions**. Published literature does not supply an exact individual William threshold for these logged signals.

## Metric definition and stable API

Full usage and signal contract are in `tools/odyssey_sim/perception/README.md`; signatures are also in `metric.py`'s module docstring. NumPy only, offline, no controller code changes.

- `perceptible_roughness(dt=..., angle_deg=..., angle_rate_deg_s=None, lat_accel=None, hand_torque_nm=None, command=None, valid=None, thresholds=None, window_s=10., hop_s=5.)`
- `from_simulator(states, dt=..., command=None, **kwargs)`
- `sluggishness(dt=..., actual=..., reference=..., threshold=..., max_lag_s=4.)`
- `Thresholds` frozen dataclass.

The main score uses 0.5–1, 1–2, 2–5, 5–10 Hz. The 0.1–0.5 Hz band is reported separately because a reference is needed to separate curves/wander/unwind from unwanted roughness. Seven-to-twenty-second wander often lies below even that band; retain lane tracking and wander metrics. Excluding slow intended motion from this smoothness scalar is not permission to ignore the highest-priority wander goal.

Linearly detrend each Hann window and calculate one-sided PSD. Prefer wheel displacement PSD derived from rate `P_rate/(2πf)²`; this avoids differentiating 0.1° ticks. Threshold-normalized band energy is `q_cb² = integral(P_c(f)/T_c(f)² df)`. Excess `E_cb = max(q_cb²−1,0)`. Sum scored bands within channel; take the maximum channel energy per window to avoid counting the same wheel/body motion twice. Return `roughness = sqrt(mean(E_window))` plus p95 window score, physical RMS and threshold ratios for each channel/band, coverage, version, thresholds and full windows.

Defaults:

| Channel | Screening RMS threshold |
|---|---|
| Wheel <3 Hz | max(0.1°, .14 × absolute mean wheel angle) |
| Wheel >=3 Hz | smaller of above and the angle equivalent of 0.2 m/s² rim tangential acceleration, assumed radius .19 m |
| Body | max(.05 m/s², .3 m/s³ / (2πf)) |
| Calibrated hand torque | max(.1 N·m, .15 × absolute mean torque) |

The high-frequency wheel floor is a background allowance, not an absolute sensory threshold. Manual routes 1a4/1a5 provide 25/26 eligible ten-second speed>=8 m/s windows without requiring lateral engagement. Their 5–10 Hz rim-acceleration p99 is .145/.183 m/s²; median .084/.089. At 2–5 Hz p99 is .143/.246. Thus the initial .03 floor was well below ordinary manual-drive background. This background combines sensor/road/driver motion, not a measured sensor noise floor. The .2 choice is provisional and sensitivity-tested.

No hand channel is scored unless calibrated N·m data is explicitly provided. Missing hand coverage is unknown, never a claim that hand roughness is zero. A scalar is insufficient without its coverage metadata. An index of zero means below these operational floors, not physiologically undetectable.

`sluggishness` returns an independently estimated nonnegative lag plus supra-threshold mismatch RMS/integral in the chosen units. With model-request reference it cannot see planner delay. The synthetic 0.8 s delayed unwind is recovered as 0.8 s while slow smooth motion does not inflate roughness. This provides the requested perceived-event calibration shape without inventing a timestamped William rating.

## Validation, iterations, and failures

### Generation 0: raw reversals / raw lateral jerk

Rejected as an acceptance gate. It scores commands before the nonlinear plant, counts tiny zero crossings, and lets differentiated high-frequency noise dominate. Kept only as diagnostic. A rising reversal count is a prompt to inspect physical outputs, not proof of worse feel.

### Generation 1: output bands, initially too sensitive

Initial `metric.py` existed early in the session. It scored the entire 0.1–10 Hz range, used a .03 m/s² high-band rim floor and the same remaining thresholds. It reported aggregate roughness 3.53 / 4.34 / 4.70 on 1a7/1aa/1ab (exact saved values in `perception_attempt1.json`). Median wheel excess was positive on these drives. Band inspection showed that slow normal curves and the 5–10 Hz channel dominated. Manual-route background exceeded the .03 floor. This failed the broad negative calibration point and identified a modeling mistake, rather than evidence William was somehow missing obvious jerk.

### Generation 2: output screening plus independent slow/reference costs

Retained API, excluded 0.1–0.5 Hz from the primary scalar and raised rim background allowance to .2 m/s² using manual data. Did not change command signals or fit a score to force NNLC to win. Most normal windows now score zero, but occasional output events remain for trace review.

| Route | Eligible windows | Roughness | p95 window |
|---|---:|---:|---:|
| 18c | 184 | .160 | .126 |
| 19b | 35 | .104 | .222 |
| 19c | 244 | .433 | .848 |
| 1a3 | 77 | .421 | 1.110 |
| 1a7 | 65 | .309 | .823 |
| 1aa | 130 | .187 | .383 |
| 1ab | 143 | .281 | .272 |

Manual-only 1a4/1a5 have no eligible *engaged* scoring windows, so their result is `None`, not zero. They are used only for background analysis above. Overlapping windows are not independent observations; p95 can be smaller than aggregate RMS because rare events dominate RMS.

### Controller provenance: all six questioned drives had NNLC configured on

Checked `initData.params.NeuralNetworkLateralControl=1`, `EnforceTorqueControl=0`, and `carParamsSP.neuralNetworkLateralControl.model.name=HONDA_ODYSSEY` for 18c, 19b, 19c, 1a7, 1aa, 1ab. `LagdToggle=1` in those snapshots. Metadata caches retain segment evidence. These are configuration snapshots, not proof an onroad toggle never changed. There is no stock-driving control group in this set merely because a route is older. Treating 18c/19b/19c as stock would be a false A/B.

### Matched NNLC / stock v0 counterfactual

Used existing `nnlc_<route>.npz` replay outputs, negated controller output to CAN sign, stepped the **same fitted plant** with the same speed and initialized lateral acceleration, and discarded the first 10 s of each 20 s block. Scored the remaining 10 s; selected eligible speed>=8 m/s, active, no override, no gaps. First 15 / 20 / 20 eligible blocks on 1a7/1aa/1ab. Wheel/body outputs only; no simulated hand model.

| Route | Windows | Stock reversals/s | NNLC reversals/s | Mean stock window roughness | Mean NNLC window roughness |
|---|---:|---:|---:|---:|---:|
| 1a7 | 15 | 1.44 | 2.23 | .104 | .055 |
| 1aa | 20 | 1.58 | 2.47 | .040 | .000 |
| 1ab | 20 | 1.27 | 2.07 | .054 | .000 |

The scores shown are means of per-window rooted scores, not an aggregate route RMS. The fitted plant supports lower felt-output roughness despite more reversals. This is open-loop propagation of replay commands receiving the same recorded feedback, not a full controller/road closed loop and not a real A/B. It cannot establish improved tracking, individual perceptibility, or hand comfort. Plant friction may suppress output excessively; lane-loop and alternative-plant checks still matter. Do not wire a raw replay command directly into this metric as if it were output motion.

### Sensitivity and slow-event calibration

At half/default/double physical floors, route roughness is 1.317/.309/.068 (1a7), .795/.187/0 (1aa), 1.063/.281/0 (1ab). Default zero-window fractions are 84.6%, 93.1%, 93.0%; half-floor fractions 33.8%, 57.7%, 42.0%. Absolute perception conclusions are therefore threshold-sensitive. Relative comparisons should be reported over this sweep, never reduced to a binary default pass/fail. Across the 55 matched counterfactual windows, mean stock/NNLC roughness at half/default/double floors is 0.215:0.119 / 0.063:0.015 / 0.005:0.000 (stock:NNLC). NNLC remains lower in this sensitivity check; at double floors NNLC is zero and stock is near zero (.005).

Selected 23 unlabelled exit-like five-second decays on 1aa/1ab: starting desired acceleration >.6 m/s², falling to <.2, eligible active windows, both desired and yaw-derived actual identically low-pass filtered at .5 Hz. The global mismatch lag estimate has a typical ~.29 s scale, with individual cases 0–.41 s. It omits the already-slow model request and does not replace the plant's command-step lag measurement. These are candidates for comparing road-relative unwind, not William-rated examples. `perception_events.json` has exact monotonic timestamps and errors. The perceived slow-exit example cannot be conclusively classified without identifying its event/reference; this remains an explicit validation limit, not a reason to abandon useful output scoring.

### Held-pulse recovery attempt: not usable as a causal experiment here

The current `odyssey_eps_report` maneuver schedule did not match recovered 1a3 held-pulse runs: the helper produced no scheduled pulses. Extracting logged `probe` plateaux directly found 14–18 instances per .02–.08 level, mostly .3–.4 s; command deltas did not consistently equal probe levels and aligned wheel deltas were non-monotonic/mostly negative. This is **not** a valid isolated held-input response. Do not use these rows to fit human thresholds or claim a deadband measurement. They are retained under `perception_events.json` as failed exploratory data. Any future pulse estimate must recover the logged hold schedule and matched zero-pulse controls from the logged maneuver state, rather than imposing today's suite.

## Checks and handoff

- Eight public behavior checks pass with `.venv/bin/python tools/test_runner.py tools/odyssey_sim/perception/test_metric.py`: output-free command chatter, amplitude separation, low-frequency curve isolation, tiny high-frequency body jerk, absent eligible data, simulator unit mapping, delayed unwind, bad input.
- `.venv/bin/ruff check tools/odyssey_sim/perception` passes.
- API works in the repo NumPy-only venv; analysis uses the shared scipy venv. No scipy dependency in metric.
- `from_simulator` consumes `angle_deg`, `angle_rate_deg_s`, `lat_accel`. **Plant `steering_accel` is an acceleration-equivalent angle proxy in m/s², not deg/s².** It is deliberately ignored.
- Use identical valid windows, coverage and thresholds for candidate comparisons. Do not compare noise-free simulator absolute scores directly to noisy logged scores. Inspect the event traces, p95, physical band RMS, and separate tracking/wander/lag costs.
- Research is bounded by unknown hand torque scale/contact, lack of subjective event labels, nonlinear closed-loop identification bias, and no large-ripple eligible stratum. A definitive causal transmission percentage is not established.

Reproduction scripts and JSON artifacts are local to this scratch directory: `perception_extract.py`, `perception_analyze.py`, `perception_counterfactual.py`, `perception_sensitivity.py`, `perception_events.py`, with correspondingly named JSON outputs; `perception_attempt1.json` preserves the failed first metric. Use `PYTHONPATH=$PWD .venv/bin/python` for extraction/counterfactual/sensitivity, and `scratchpad/wander/venv/bin/python` for spectral analysis/events, from the repo root. No changes to runtime or other harnesses were made; each worker imports the stable module.
