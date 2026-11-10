"""Write the requested report exception; no runtime edits or forecast rewriting."""

# Long Markdown prose literals preserve readable generated paragraphs.
# ruff: noqa: E501
import hashlib
import json
from pathlib import Path
from tools.odyssey_sim.shake.identify3 import OUT, PLOTS, save


def main():
  timing = json.loads((OUT / 'timing3.json').read_text())
  fits = json.loads((OUT / 'yaw3.json').read_text())['fits']
  summary = json.loads((OUT / 'summary3.json').read_text())
  start = json.loads((OUT / 'manifest_start3.json').read_text())
  end = json.loads((OUT / 'manifest_end3.json').read_text())
  lines = [
    '''# Shake research, pass 3: timing fixed, first physical link still fails

**No: correcting the timing does not yet make this model match the van, and the simulator still cannot be trusted to reproduce the US-169 shake.** The raw gyro confirms the lead's finding: on route 1ac segment 7, pose-filter yaw follows raw yaw by **80 ms**, with correlation **0.994**. The native CAN wheel angle leads raw yaw by **70 ms** and pose yaw by **150 ms**. Those are correlation peaks over the full segment, rather than the lead's shorter window/single-axis estimate of 55/140/85 ms. An 80 ms measurement delay contributes 28.8° at one oscillation per second and 43.2° at 1.5 times per second; it must not become fictitious tyre or inertia dynamics.

But the corrected wheel-to-yaw model passes only **130/321 coherent check bins (40.5%)**: **37/93 at 25–45 mph**, **93/228 above 45 mph**. Another **967 bins are uncovered**, not passing. The error changes with steering amplitude and route as well as frequency. One additional delay cannot repair a gain error. The brief explicitly says to stop here when this link fails: **no forward column fit, rebuilt disturbance, closed-loop stock/S3/S5 gates, new road forecast or quiet-bump tuning was run.** The prior locked forecast remains unchanged.

Your reported US-169 push remains ±0.25 m/s² (±20 N on 80 kg) typical and ±0.50–0.625 m/s² (±40–50 N), about 1.3 oscillations per second and 2.7 direction changes per second. This pass does not replace that road evidence with a fitted simulator result. For your actual 75 kg, multiply every quoted force by 0.9375.

**Recommendation:** finish the already planned isolated pulse measurements with actual final CAN input and timestamped raw sensors retained. Use those as independent excitation to distinguish small-angle measurement/compliance effects from tyre response. Keep the failed simulator out of pulse optimization. The old held-pulse suite is retired by BRIEF_PULSE.md; do not revive pass 2's 4-minute-36-second proposal, whose lane-keeping input contaminated the experiment.

## Scope, data and provenance
'''
  ]
  lines.append(
    f"Ran in `~/Code/odyssey-pilot`, branch `{start['branch']}`, HEAD `{start['head']}`. No branch switch, worktree, commit, push, device or van access. Only pass-3 files in `tools/odyssey_sim/shake/`, assigned `scratchpad/shake/`, the off-repo plot directory, and this explicitly requested report were authored. No other worker's file was edited or reverted. Start/end manifests contain full dirty status and controller-source hashes; raw logs were read locally. No runtime controller is imported or simulated in this stopped pass.\n\nStart status:\n```text\n{start['status']}```\nEnd status:\n```text\n{end['status']}```\n"
  )
  changed = [p for p, h in start['sha256'].items() if end['sha256'].get(p) != h]
  lines.append(
    f"Controller files whose hashes changed during this fit: {', '.join(changed) if changed else 'none'}. This is a boundary comparison, not a claim to freeze the concurrently edited checkout. `sources3.json` records analysis/locationd hashes and native-cache provenance.\n"
  )
  lines.append('''
All thirteen routes from 174 through 1ae used by pass 2 were re-extracted from rlogs. The route split is unchanged: fit all except entire 19c, manual 1a5, 1ad, and exposed check 1ae. The other manual route, 1a4, trains. These checks have already been examined by earlier passes; none is new untouched proof. No later-drive data was opened. There are 286 eligible continuous 20-second windows, with 66 lower-speed and 128 higher-speed training windows. Gyro gaps over 50 ms reject a window. Existing speed/activity/blinker/segment splitting is inherited from `identify2.pieces`; hand flags are not blanket exclusions. Native angle gaps have route-wise median window maxima of 13–23 ms; no long wheel gap is silently treated as a trajectory.

## Sensor frames and verification

The logged `extrinsicsCalibration` is this software generation's name for the brief's `liveCalibration`. At each sensor timestamp, take the latest logged calibration, map native sensor axes to device axes **[-z, -y, -x]**, then multiply by **rot_from_euler(rpyCalib).T** to obtain calibrated vehicle axes: forward, right, down. This exactly follows local `locationd.py` and `locationd/helpers.py`; there is no correlation-selected axis or fitted rotation. Before the first calibration sample in a route, its first available sample is used. Calibration is a camera/road-frame estimate, not a survey of the body's mounting angles. All thirteen routes have the same identified source, `lsm6ds3trc`; sources are retained rather than mixed, and bmx055 would be excluded as locationd does.

Stationary speed below 0.1 m/s provides a gravity sign/size check. Across routes, median acceleration-vector norms are 9.765–9.912 m/s² (781.2–793.0 N gravity reference on 80 kg, **not sideways felt push**). Vehicle z is upward specific force, approximately −9.81 m/s² (−784.8 N gravity reference), as expected. Example 1ae median: [0.055, −0.039, −9.809] m/s² ([4.44, −3.14, −784.72] N on 80 kg). Other stationary medians have horizontal components up to about 0.79 m/s² (63.2 N), compatible with parked grade/bank/calibration and sensor bias: the logs do not prove a level pad or precision roll alignment. This validates axis/sign/scale for yaw work, not a windshield-to-seat acceleration model.

Raw gyro is uncalibrated and has a route yaw DC bias of 0.0717–0.0845 rad/s. Subtract the stationary median when at least 500 stationary gyro samples exist; only route 19b uses the route-median raw-minus-pose DC instead. That alternative borrows **only a constant**, never the pose filter's time-varying signal. The 0.3–3-times-per-second spectral fit removes DC anyway. Temperature drift, imperfect stationary leveling and constant bias affect the long-curve check; no raw DC is silently called vehicle turning.

After bias removal, the low-frequency (below 0.2 times per second) raw/pose comparisons have route-median correlations 0.997–1.000 and pose/raw slopes 0.989–1.008. Known sustained turns have the same yaw sign as speed × curvature from the negative wheel angle. Their **size** is checked with the steady bicycle relation and an estimated understeer term below; steering-derived curvature is not an independent gyro ground truth. No video-based turn adjudication or independent surveyed curvature was available. No roll, seat or head-force identification is claimed.

## Measured timing, with intervals

All numbers below are milliseconds. Brackets are the empirical 2.5–97.5 percentiles across eligible 20-second windows with lag correlation above 0.7; they are **spread intervals, not confidence intervals for absolute physical delay**. Peaks are resolved in 10 ms steps. Raw gyro and accelerometer use their sensor timestamps, eliminating publication lag from the tyre fit. CAN has host reception/publication timestamps, not ECU acquisition timestamps.

| Route | Pose after raw yaw | Native angle after integrated native rate | Motor magnitude after absolute CAN command |
|---|---:|---:|---:|
''')

  def ms(q):
    return 'unexcited' if not q else f'{q[1] * 1000:.1f} [{q[0] * 1000:.1f}, {q[2] * 1000:.1f}]'

  for n, r in timing.items():
    lines.append(f"| {n[-3:]} | {ms(r['pose_lag_s'])} | {ms(r['angle_after_integrated_rate_s'])} | {ms(r['motor_after_abs_command_s'])} |\n")
  lines.append('''
The gyro's route-median publication delay is roughly 2 ms, with per-route sample spreads retained in `timing3.json`; the accelerometer is roughly 1 ms. The native gyro timestamp, not publication time, is used for resampling. The pose lag is 70–80 ms, with occasional window peaks up to 90 ms. Pose yaw would require its own explicit delay in a future observation model.

Angle/rate mismatch is amplitude/route dependent: automated driving often gives angle 40–100 ms after integrated rate, while the manual routes are about −10 to 0 ms. Several windows hit the ±150 ms search edge. Thus this is **not a clean fixed ECU delay estimate**; rate filtering, rate integration and angle quantization contribute. Native-angle versus cached carState-angle peaks are 0 ms at 10 ms resolution. Direct CAN reception avoids an additional carState scheduling assumption. We fit an explicit bounded **relative yaw-minus-angle observation delay**, allowing −100 to +100 ms. This is an analyst sensitivity bound, not a measured 95% interval or proof that all ECU delay lies inside it. Fitted route-bootstrap intervals are given below. A negative fitted delay means the angle observation is effectively later than gyro, not that the van anticipates steering.

Motor-magnitude lag after absolute command is generally 0–20 ms, with negative/noisy windows and broad tails on 1ac/1ad. It mixes assist dynamics, unsensed sign at reversal, driver assist and CAN scheduling. It cannot separately identify motor transport delay and first-order lag. Final CAN command follows the logged actuator request by route-median 10 ms on earlier automated drives, 40–45 ms on 1ac–1ae, with window spreads approximately 0–100 ms. Slew limiting shapes this comparison; do not treat that whole number as transport delay. Full intervals and native sampling intervals are in `summary3.json`. These links are measured diagnostics only: because link 1 fails, no downstream physical fit absorbs them. In a continued model each command/motor observation must retain a distinct bounded delay; none can inherit the tyre delay as a substitute.

## Corrected wheel-angle-to-yaw fit and gate

Same four-state single-track model with front/rear tyre relaxation as pass 2. Use native CAN wheel angle (negated into curvature-positive radians), calibrated raw yaw at sensor time, and measured speed. Fixed inherited mass 2163.35 kg, axle distances 1.35/1.65 m, steering ratio 14.35; actual load, tyre pressure, effective steering compliance and road bank remain unknown. The physical transfer is multiplied by `exp(-j*2*pi*f*relative_delay)`. No yaw amplitude scale is fitted. Roll remains outside the yaw path.

Each speed band uses seeded differential evolution (population 60, up to 120 generations), bounded least squares, then 60 whole-training-route bootstrap refits. Five-second Hann records, 50% overlap, yield bins every 0.2 times per second; scoring starts at 0.4, not an invented 0.3 bin. Fit only coherence ≥0.7. Stiffness bounds are 20–600 kN/rad, inertia 1000–16000 kg·m², relaxation 0.02–6 m. Intervals below are conditional route-bootstrap percentiles under this rejected model.

| Speed | Parameter | Estimate | Conditional 95% interval |
|---|---|---:|---:|
''')
  names = ['Front axle stiffness, N/rad', 'Rear axle stiffness, N/rad', 'Yaw inertia, kg·m²', 'Relaxation length, m', 'Relative observation delay, s']
  for f in fits:
    for i, n in enumerate(names):
      lines.append(
        f"| {'25–45 mph' if f['speed'] == 0 else '>45 mph'} | {n} | {f['parameters'][i]:.5g} | {f['interval95'][0][i]:.5g} – {f['interval95'][1][i]:.5g} |\n"
      )
  lines.append('''
The front/rear stiffness and inertia intervals overlap across bands, satisfying a literal interval-overlap check, but front stiffness point estimates still differ by 69% and both inertia bootstrap distributions reach the 1000 kg·m² floor. Inertia remains weakly identified; overlapping broad intervals are not proof of a common physical van. These results replace neither published vehicle dimensions nor a torque calibration.

The steady relation is `theta/ratio = L*yaw/v + K*v*yaw + route steering-zero bias`. Long-curve samples are nonoverlapping ten-second records, yaw magnitude >0.006 rad/s, yaw SD under 20% of mean, speed SD below 0.5 m/s. Training has 26 curves and checks have 8; route biases are fitted independently of dynamic parameters. Measured K is **0.00349 s²/m [0.00258, 0.00433]** in training, **0.00269 [0.00233, 0.00370]** in checks. These are curve-bootstrap intervals, with sparse curves and gyro-DC/road-bank limitations; the training wheel residual is 0.99° RMS, check 0.18°. This is a conditional steady-gain check, not independent precision understeer metrology.
''')
  for f in fits:
    lines.append(
      f"\nDynamic-fit K, {'25–45 mph' if f['speed'] == 0 else '>45 mph'}: **{f['understeer_s2_m']:.5f} s²/m**, route-bootstrap interval {f.get('understeer_interval95_s2_m', [])}.\n"
    )
  lines.append('''
The lower-speed point estimate 0.00653 fails to match either measured steady-gain interval; the highway point estimate 0.00362 lies inside both. This additional gate is not established across both bands. Constant gyro-bias uncertainty limits precision, but cannot turn the already failed frequency-response gate into a pass.

| Check route | Speed | Pass / coherent bins | Median predicted/measured gain | Median phase error |
|---|---|---:|---:|---:|
''')
  for k, r in summary['residual']['by_route_speed'].items():
    if r['n']:
      n, s = k.split('/')
      lines.append(
        f"| {n[-3:]} | {'25–45' if s == '0' else '>45'} | {r['passed']}/{r['n']} | {r['gain_2_50_97'][1]:.3f} | {r['phase_2_50_97_deg'][1]:+.1f}° |\n"
      )
  lines.append(f'''
A bin passes only if gain is 0.85–1.15 **and** absolute phase error ≤15°. The gate requires all covered bins, not a favorable median. Across 321 covered checks, gain's empirical 95% spread is **0.379–1.527**, phase **−39.3° to +28.4°**. This is not a small uniform residual time shift. Pass 2 had 100/298, but different raw-output coherence changes coverage; 130/321 is not a clean paired improvement statistic.

![Corrected held-route yaw gain and phase]({PLOTS / 'yaw_residual3.png'})

### Exactly what remains in the residual

Frequency: median gain is near 1 through 1.4 times per second, but route/window spreads are wide; median gain falls to 0.91 at 1.8 and 0.80 at 2.6 times per second. Median phase moves from +4.6° at 0.4 to about −10° at 2.4 times per second. Only 5/16 bins pass at 2.0, 2/8 at 2.6, and 1/2 at 3.0 times per second; that sparse upper-band coverage cannot establish a precise resonance. At 1.0 times per second, 14/35 pass despite median gain 1.016 and phase +5.9°: the median hides route dependence.

Speed: 25–35 mph passes 19/46 (median gain 0.926), 35–45 passes 18/47 (0.958), 45–55 passes 16/65 (1.180), above 55 passes 77/163 (0.968). The 45–55 group tends to overpredict and has median phase +9.9°. These are observational groups, not matched speed experiments.

Amplitude: below 0.15° bandpassed wheel RMS, gain is **0.580**, pass **10/42**; 0.15–0.3° gives **0.880**, **12/34**; 0.3–0.6° gives **1.015**, **73/144**; above 0.6° gives **1.103**, **35/101**. This is measured yaw being larger than the model predicts at the smallest angle swings. Angle quantization (0.1°), sensor error in the excitation, steering compliance, tyre nonlinearity and feedback bias are candidates, not established causes. These bins come from overlapping spectra, not hundreds of independent physical repeats.

Route: highway 19c is underpredicted (median 0.812, 20/82 pass), manual 1a5 overpredicted (1.224, 5/40), 1ad substantially closer (1.028, 68/106). Lower-speed 1ae is 0.934, 29/75. The route dependence survives the explicit observation-delay term. A single bandwise linear tyre model does not explain every route/amplitude. The residual tables include every tested frequency, speed, amplitude and route in `summary3.json`; each individual bin with window time and coherence remains in `yaw3.json`.

## Downstream gates and forecast

| Gate/task | Pass-3 result |
|---|---|
| Frame/sign/scale and pose latency | Supported at yaw-analysis accuracy; precision bank/roll and absolute ECU latency remain unidentified |
| Wheel-to-yaw gain/phase | **FAIL: 130/321**, with 967 uncovered |
| Across-speed physical intervals | Stiffness/inertia intervals overlap; inertia reaches lower bound; not a validated shared parameter set |
| Steady-gain consistency | Lower-speed dynamic K 0.00653 vs measured training 0.00349, check 0.00269; **not met across bands** |
| Forward column | Not run: first-link stopping condition |
| Whole-chain open loop | Not run; no pass-3 gate number |
| Closed-loop stock / S3 / S5 | Not run; no pass-3 controller gate numbers |
| Causal factorial, quiet dead zone, PWM optimization | Not run; no supported causal or design conclusion |
| New stock off / kick 3% / bump 6% forecast | Not issued because first-link stop applies before the forecast stage |

`forecast_locked2.json` and `forecast_lock2.json` are preserved byte-for-byte; verification hashes are in `sources3.json`. No pass-3 forecast file with invented intervals is created. The next drive remains the only prospective proof, but it cannot validate a pass-3 forecast that was never justified or issued.

## What the new pulse test must deliver

The isolated 100 Hz shots in BRIEF_PULSE.md provide independent applied-input timing, whereas the retired suite did not freeze lane keeping. Use randomized sign/order, both speed bands, both reversal histories and shams, with a whole later route/session reserved from fitting. Short shots alone cannot validate steady response at 0.3 times per second: retain normal long turns and intervening driving too. No increase in pulse size or road-use change is proposed here.

Still unidentified: command-to-motor transport versus lag, reversal sign of magnitude-only torque, small-signal angle/rate filtering/compliance, forward column inertia/friction/presliding and slow road-load separation, tyre relaxation/inertia tradeoff, and a disturbance model independent of measurement error. Absolute column torque/inertia/friction units also require a torque calibration; pulses cannot manufacture that scale. Seat/head/roll transfer remains outside these yaw measurements.

For direct simulator ingestion the report JSON must include **shot-level time series or paths to lossless arrays**, not just per-condition means:

- Schema version, route/segment/shot IDs, software and calibration hashes, clock units, completed/aborted status and reason, shot phases/timestamps, planned condition and randomized repetition, speed band, actual baseline, prior wheel direction/reversal status, sham ID, saturation/slew/clipping flags, and train/check-session eligibility.
- Native monotonic timestamp arrays for raw gyro/accelerometer **sensor time and publication time**, source IDs and raw three-vectors; calibration rpy/status at native timestamps; native CAN wheel angle/rate, motor magnitude, actual final command and bus/source IDs. Preserve native sensor precision and sample gaps. Carry pose yaw separately as a delayed observation. Do not merge sensor streams first and discard their timestamps.
- Requested command, post-test command, post-carcontroller/carOutput/final CAN command separately; pulse height, achieved widths, period, duty, area and signed baseline; speed, steering-active, sustained driver torque, abort events, lane offset/heading/confidence, desired curvature and recovery intervals. Identify actual lane-control contribution during each shot, including any nonzero handover action.
- Per-shot measured wheel displacement from integrated rate plus the absolute angle anchor; wheel-rate trace, raw-yaw trace and measured response onset/peak/RMS/tail; baseline/sham subtraction method and repeat counts. Body lateral acceleration must state its definition and delay: speed × raw calibrated yaw for identification, separately speed × pose yaw for historical scoring. Every force should carry mass explicitly (75 kg for the pulse UI/report, plus 80 kg comparison when consumed here).
- Aggregate completed-shot counts, direction/speed/history strata, sham variability, uncertainty intervals and the held-out session definition; unresolved intervals must be null with a reason, not zero. Three repetitions are the planned minimum per condition, not a guarantee of identifying a threshold. Retain pulse trains at 0.25/0.5 s periods as known input waveforms, not “equivalent average torque” until measured.

Exact next step: load those timestamp-preserving independent shots, estimate sensor/command response timing against shams, test whether the wheel-to-yaw amplitude dependence persists with rate-integrated angle and measured input as an instrument, then refit the tyre link with the same held-session gain/phase and steady-gain checks. Only after it passes should the forward column/disturbance/closed-loop chain continue.

## Verification and reproduction

Ten regression checks pass through the repository runner: eight retained pass-1/pass-2 mechanics checks and two new independent frame/timing checks. Without scratch SciPy, both new tests skip cleanly (2 skips). Ruff is clean for all five new analysis/test/report modules at the verification point. The residual plot was rendered and visually inspected; it is outside git at the absolute path above. Code tests check transformations, lag sign, numerical and inherited mechanics invariants; the road model gate still fails.

Run with `PYTHONPATH=$PWD:<assigned scratch>/shake/site` and `MPLCONFIGDIR=<assigned scratch>/shake/mpl`, using `.venv/bin/python -m tools.odyssey_sim.shake.timing3 <route names>` for native extraction, then `...identify3`, `...diagnostics3`, `...report3`. Native caches are `<assigned scratch>/shake/*_native3.npz`; complex spectra are `spectra3.npz`. Re-running identification replaces pass-3 results only. `report3` regenerates this explicitly requested report. Do not rewrite any locked forecast after opening the next-drive logs.

Self-review checked native sensor axes against locationd, calibration transpose, DC bias versus delay, delay sign with a synthetic 90 ms shift, no IMU-source blending, route-held separation, gain/phase convention, coverage counts, bound-limited inertia and steady-gain limitations. No outside literature or new published constants were used: physical equations/constants are inherited from `identify2.py`/`physics.py` and the prior reports. In particular, passive cross-correlation spreads are not mislabeled as absolute ECU latency confidence intervals. No work remains running at handoff.
''')
  Path('tools/odyssey_sim/reports/report_shake3.md').write_text(''.join(lines))
  save(
    'sources3.json',
    {
      'head': start['head'],
      'sha256': {
        str(p): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [
          *OUT.glob('*3.py'),
          OUT / 'forecast_locked2.json',
          OUT / 'forecast_lock2.json',
          Path('openpilot/selfdrive/locationd/locationd.py'),
          Path('openpilot/selfdrive/locationd/helpers.py'),
        ]
      },
      'cache_provenance': 'Native rlog local extraction; original sensor and host times retained; full route IDs in NAMES/glob. No new-drive data.',
    },
  )


if __name__ == '__main__':
  main()
