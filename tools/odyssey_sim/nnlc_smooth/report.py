# ruff: noqa: E501
"""Build the research receipt from frozen trial outputs, including every failure."""

import hashlib
import json
from pathlib import Path
import numpy as np
from tools.odyssey_sim.nnlc_smooth.experiment import OUT
from tools.odyssey_sim.nnlc_train import SCRATCH as S, MODEL_DIR


def mean(rows, key):
  return float(np.mean([r[key] for r in rows if r.get(key) is not None]))


def main():
  sel = json.loads((OUT / 'selection_scored.json').read_text())
  test = json.loads((OUT / 'test_scored.json').read_text())
  replay = json.loads((OUT / 'replay.json').read_text())
  names = ['Stock V0+NNLC', 'V1', 'S3 (20 ms FF)', 'S4 (gain 1.4, 50 ms error)', 'S5 (gain 2, 50 ms error)', 'V2 (offline causal adapter)']
  lines = [
    '# Odyssey learned-steering research receipt',
    '',
    '**Recommendation: advance S3 as the conservative review candidate, and retain S5 as the stronger straight-tracking research arm. V1 and V2 must no longer be rejected merely for torque reversals.** All candidates remain default-off; no road/device changes, commits, branch switches or pushes occurred.',
    '',
    '## What this establishes',
    '',
    'The owner-trained inverse model has useful closed-loop tracking performance. A tiny causal feedforward filter preserves V1 tracking while reducing command activity. Stronger torque-space feedback improves straight and curve tracking further, with a measurable output-motion tradeoff. This is evidence for an offline candidate, not a certified improvement in lane wander or driver feel.',
    '',
    '## Research used',
    '',
    '- [comma Controls Challenge / RL controls](https://blog.comma.ai/rlcontrols/): evolutionary parameter search directly optimizes closed-loop performance; their feature-rich feedback controller used six tuned parameters. I used deterministic grid plus seeded random black-box search over controller shaping, rather than retraining inverse labels as though their MAE were the driving objective.',
    '- [jonoomph neural controller](https://github.com/jonoomph/controls_challenge): documents temporal state and torque-history inputs; relevant to preserving the actual causal runtime contract. Its performance claims are its author’s, not a demonstrated Odyssey transfer.',
    '- [omarrayyann RL controller](https://github.com/omarrayyann/controls_challenge): a learned closed-loop-policy lead; no weights or claimed results transferred here. PPO/model-based MPC are alternatives, but the available dynamics and perception models remain uncertain enough that compact, bounded controller tuning is the more reviewable first rep.',
    '- [SAE 2015-01-2729](https://saemobilus.sae.org/papers/active-return-center-control-strategy-steering-wheel-sensor-electric-power-steering-system-2015-01-2729): return-state identification from wheel velocity/torque and double-loop return compensation. [SAE 2008-01-0499](https://saemobilus.sae.org/papers/sliding-mode-return-control-electric-power-steering-2008-01-0499): friction-robust return control and assist/return switching. These abstracts motivate state-conditioned unwind; neither validates adding an unconditional centering torque while lane keeping.',
    '- [US6408234B1](https://patents.google.com/patent/US6408234B1/en) is specifically EPS hysteresis compensation, not an empirical Odyssey return-to-center result. Its distinction between direction/history-dependent response and instantaneous torque supports treating raw command chatter separately from wheel motion.',
    '- Local sources: ODYSSEY.md; report_van_nnlc.md; report_calib.md; BRIEF_COMMON.md; report_perception.md and perception/metric.py. Human-threshold literature is assessed by the perception report. No published sunnypilot NNLC smoothing method was established by this work; absence of a method is not proven.',
    '',
    '## Data, objective and evaluation contract',
    '',
    'No new supervised weights were trained. V1’s original public pretraining and owner adaptation are reused. Adaptation already included 19c, so 19c is not a held-out controller result here despite being a plant holdout. All tuning uses a7 only. Three arms were frozen in selected.json before aa/ab evaluation. The later V2 run is an untuned reference with its existing weights/history adapter, not a model selected on aa/ab.',
    '',
    'Primary evaluator: calibrated schema-v2 William plant, logged-plan mode, actual V0/NNLC, 100 Hz updates, 10 Hz logged plans, per-frame tuning/delay/pitch, 0.1-degree feedback quantization and Honda 3 units/s applied-command slew. Public sensitivity: original public schema-v1 steering dynamics with the same calibrated controller front end and slew; this is deliberately a different, uncalibrated public plant, not a second validated v2 fit. Body-stage sensitivity remains approximate.',
    '',
    'Selection: two a7 windows, 60 s each. Final test: two aa windows of 60 s and six ab windows of 30 s, selected by the calibration harness’s uninterrupted active/no-driver and lane-confidence rules. First 10 s warmup excluded. Eligibility excludes <=5 m/s, blinker/driver activity and gaps. Results are this selected sample, not route-wide prevalence. Tables average eligible-window metrics equally; raw n and traces are retained for pooled analyses. Cost = 100×mean squared lateral-acceleration error, not the full Controls Challenge 50×cost+jerk objective.',
    '',
    'Straight following has priority: inspect |planned accel|<0.1 separately, then total tracking, output roughness and the alternate plant. The scalar 3×straight-cost + total-cost was inspected as a ranking aid, not used as a stand-alone safety gate. Families cover reduced gain/friction, causal error filtering, friction filtering, FF filtering, jerk-feature suppression and command slew. Random search seed 20261109. We retained Pareto alternatives instead of optimizing away William’s priority with a reversal penalty.',
    '',
    '## Held-out aa/ab: motion and commands',
    '',
    '| Arm | Plant | Tracking cost | Straight cost | Mid-curve cost | Raw flips/s | Output roughness |',
    '|---|---|---:|---:|---:|---:|---:|',
  ]
  for g in sorted({r['generation'] for r in test}):
    for p in ('william', 'public'):
      a = [r for r in test if r['generation'] == g and r['plant'] == p]
      lines.append(
        f"| {names[g]} | {p} | {mean(a, 'cost'):.4f} | {mean(a, 'straight'):.4f} | {mean(a, 'mid'):.4f} | {mean(a, 'flips'):.3f} | {mean(a, 'roughness'):.4f} |"
      )
  lines += [
    '',
    'Roughness is the perception worker’s output-based index, not probability or an absolute comfort rating. Wheel/body channels only; hand torque is unavailable from either plant. Command flips never enter this score. Stock can score zero while tracking poorly: low activity is not sufficient. S3’s few remaining reversals do not by themselves establish discomfort, nor does a zero score prove no hand reaction.',
    '',
    '### Threshold sensitivity',
    '',
    '| Arm | Plant | thresholds ×0.25 | ×0.5 | ×1 | ×2 |',
    '|---|---|---:|---:|---:|---:|',
  ]
  for g in sorted({r['generation'] for r in test}):
    for p in ('william', 'public'):
      a = [r for r in test if r['generation'] == g and r['plant'] == p]
      vals = [float(np.mean([r['sensitivity'][str(x)] for r in a])) for x in (0.25, 0.5, 1.0, 2.0)]
      lines.append(f"| {names[g]} | {p} | " + ' | '.join(f'{v:.4f}' for v in vals) + ' |')
  lines += [
    '',
    'All perceptual thresholds except wheel radius are scaled together. The final report uses perception v2 (0.2 m/s² provisional rim allowance; primary bands >=0.5 Hz), superseding exploratory scores calculated while that worker was still editing v1. No hidden assumption that high EPS friction eliminates hand torque: the sensor retains command-correlated ripple, and these models do not predict it in calibrated N·m.',
    '',
    '### Four priorities',
    '',
    '1. **Straight/no wander:** S3/V1 substantially reduce small-plan acceleration error versus stock, with S5 the stronger straight candidate. This does not prove the 7–20 s lane loop improves: logged plans stay fixed and the calibrated lane geometry does not close. Perception’s primary bands also exclude that slow wander. Hand this result to the wander/calib3 worker as a better inner-loop candidate.',
    '2. **Low speed:** no plant validates parking/intersection operation. This evaluator excludes <=5 m/s and lane-confidence filtering further reduces neighbourhood coverage. Runtime shaping bypasses below 5 m/s; it does not fix V0’s low-speed gain or saturation. The base V1 inference is still an experimental model there. Keep the low-speed worker’s intervention separate until the lead integrates and verifies it.',
    '3. **Pre-entry:** tracking improves for changing plans and mid-curve, but the existing target does not command earlier than the logged planner. A 20 ms FF filter introduces a small causal lag, not anticipation. Future-plan inputs remain causal runtime plan signals. No claim of turning before a curve or reducing real takeovers follows from these scores.',
    '4. **Exit:** no unconditional wheel centering was added. Phase-specific accel error and lag are diagnostics below; an upstream ~3 s planned decay cannot be fixed by faithfully tracking that same decay. A separate plan-conditioned unwind module needs curve-worker evidence.',
    '',
    '### Planned-entry / exit diagnostics',
    '',
    '| Arm | Plant | rising-demand cost | falling-demand cost | mean best-fit lag (s) |',
    '|---|---|---:|---:|---:|',
  ]
  for g in sorted({r['generation'] for r in test}):
    for p in ('william', 'public'):
      entry = []
      exit_ = []
      lags = []
      for r in [x for x in test if x['generation'] == g and x['plant'] == p]:
        z = np.load(OUT / f"test_{g}_{r['route'][:8]}_{r['window']}_{p}.npz")
        slope = np.repeat(np.gradient(abs(z['target'][::10]), 0.1), 10)
        err = (z['y'] - z['target']) ** 2 * 100
        eligible = np.arange(len(err)) >= 1000
        for out, mask in ((entry, slope > 0.1), (exit_, slope < -0.1)):
          if (eligible & mask).any():
            out.extend(err[eligible & mask].tolist())
        if r['lag'].get('lag_s') is not None:
          lags.append(r['lag']['lag_s'])
      lines.append(
        f"| {names[g]} | {p} | {np.mean(entry) if entry else float('nan'):.4f} | {np.mean(exit_) if exit_ else float('nan'):.4f} | {np.mean(lags) if lags else float('nan'):.3f} |"
      )
  lines += [
    '',
    'Rising/falling means derivative of |planned accel| above/below ±0.1 m/s³, not independently labelled curve events. Lag is the metric’s global reference-conditioned best shift, which can be unreliable with weak excitation. These diagnostics do not score road pre-entry timing.',
    '',
    '## Logged-motion replay (actual runtime shaping)',
    '',
    'First 20,000 control frames per held-out route, the validated replay adapter, real V0. Stock correlation and MAE verify the inherited replay path. Candidate command MAE means difference from the historical controller, not better tracking. Measured motion is fixed across replay arms; attributing different measured roughness to them would be invalid.',
    '',
    '| Route / mph | Arm | n | Command MAE | Correlation with logged | Flips/s | command-rate RMS |',
    '|---|---|---:|---:|---:|---:|---:|',
  ]
  for r in replay:
    lines.append(
      f"| {r['route'][:8]} / {r['band']} | {r['model']} | {r['n']} | {r['command_mae']:.5f} | {r['r']:.4f} | {r['flips']:.3f} | {r['rate_rms']:.3f} |"
    )
  lines += [
    '',
    '## Generation log: every attempted configuration',
    '',
    'Cells are mean tracking cost / raw flips per second / output roughness. These are a7 selection results, not held-out scores. Parameters omitted from the compact label retain Trial defaults. Failures remain in generations.json and the native traces; duplicates were intentional cross-checks or part of the regular grid.',
    '',
    '| Gen | Trial | William cost/flips/roughness | Public cost/flips/roughness | Assessment |',
    '|---:|---|---|---|---|',
  ]
  for g in sorted({r['generation'] for r in sel}):
    a = [r for r in sel if r['generation'] == g]
    t = a[0]['trial']
    label = ', '.join(f'{k}={v:.3g}' if isinstance(v, (float, int)) else f'{k}={v}' for k, v in t.items() if k == 'model' or v not in (0.0, 1.0, 100.0))
    # Also show meaningful zero friction and jerk, otherwise omitted values hide trials.
    label += ''.join(f', {k}=0' for k in ('friction', 'jerk_scale') if t.get(k) == 0)
    cells = []
    for p in ('william', 'public'):
      z = [r for r in a if r['plant'] == p]
      cells.append('/'.join(f'{mean(z, k):.4f}' for k in ('cost', 'flips', 'roughness')))
    if g == 0:
      reason = 'Stock reference'
    elif g in (1, 16):
      reason = 'V1 reference; tracking improves, reversals alone not failure'
    elif g == 55:
      reason = 'S3 selected: preserves tracking; less command ripple'
    elif g == 49:
      reason = 'S4 selected: stronger feedback with causal damping'
    elif g == 51:
      reason = 'S5 selected: stronger straight tracking; inspect output tradeoff'
    elif g == 87:
      reason = 'Untuned V2 causal-history reference; offline only'
    elif t.get('friction') == 0 or t.get('error_gain', 1) < 0.7:
      reason = 'Tracking loses useful restoring action, especially public plant'
    elif t.get('slew', 100) < 3:
      reason = 'Less command activity but added tracking lag'
    elif t.get('error_gain', 1) >= 3:
      reason = 'Straight error improves; total error/output energy tradeoff'
    elif t.get('error_tau', 0) > 0.1 or t.get('friction_tau', 0) > 0.1:
      reason = 'Filtering adds response lag; no clear Pareto win'
    else:
      reason = 'Retained sensitivity/Pareto alternative; not selected'
    lines.append(f"| {g} | {label} | {cells[0]} | {cells[1]} | {reason} |")
  lines += [
    '',
    '## Root causes and what failed',
    '',
    'V2 is not rejected by the perceptual screen: it has lower wheel/body output roughness than V1 on William’s plant with similar tracking. It remains offline because its 26-input contract is not accepted by the runtime override, and its reused history adapter supplies a pre-NNLC controller-output proxy rather than applied-CAN history. In fact LatControlTorqueExt assigns the current base-controller output before CandidateModel.update runs; calling that actual past applied torque would be wrong. A causal applied-command-history integration and retraining/evaluation against that exact contract is the next V2 implementation step. S3 is recommended for the first bounded, reviewable 18-input integration, not because V2 has too many reversals.',
    '',
    '- Simply lowering gain or removing friction makes commands calmer while losing tracking. EPS stiction requires a restoring action; penalizing raw torque motion rewards a controller that does too little.',
    '- NNLC mixes learned feedforward, learned setpoint-minus-measurement feedback, high-accel error blending, explicit friction override and torque-space PID. The inverse trainer’s third feature was achieved jerk, but runtime uses friction/error proxies; the mismatch is real. Zeroing/scaling that input alone did not preserve robust tracking. No claim that a retrained model has identified a causal EPS law.',
    '- Filtering the complete error or friction response is causal but adds feedback lag. Slow output slew is especially costly to responsiveness. Short FF-only filtering removes frame-to-frame feedforward activity while retaining the restoring path; it is shaping at the identified path, not a broad final-actuator low-pass.',
    '- Stronger error gain with short error filtering gives better small-plan tracking than V1. The cost is output energy on some windows. Those candidates remain live alternatives rather than being rejected over command flips.',
    '- No new temporal-consistency supervised fit, differentiable-plant policy, full RL, MPC, or 250 h retraining was completed. They are not recorded as failed experiments. These 88 controller trials test the more direct closed-loop-objective path first.',
    '',
    '## Runtime integration, checks and artifacts',
    '',
    'Only shared runtime edit: openpilot/sunnypilot/selfdrive/controls/lib/nnlc/nnlc.py. Added NnlcResponseShaper, validated JSON metadata loaded only for an explicit model override, causal separate feedback/FF filtering, A/B and idle/driver/low-speed state resets. No new param. Default stock path and existing metadata-free V1 remain identical. Invalid metadata restores stock model; SafeNNTorqueModel retains finite bounded inference and sanitized-stock fallback on invalid live values. Existing PID limits bound complete command. The numerical shaper guard resets invalid state. No OdysseySteer edit or other worker directory edit.',
    '',
    'Tests: the new causal step-response/reset seam first failed collection because the runtime class did not exist, then passed. Existing NNLC tests plus four new tests pass: 39 total. Tests cover metadata absence/invalid values, bad-input reset, causal filtering and actual override construction with stock identity. Ruff passes on this task’s Python files and nnlc.py. Full production construction was exercised by calib.evaluate --controller ...:runtime_factory; its output is scratchpad/calib/nnlc_smooth_runtime.json. That evaluator’s overall lane calibration gate remains false by design; candidate tracking is not a baseline-calibration test.',
    '',
    '| Model | Host median ms | Host p95 ms | Observed host max ms |',
    '|---|---:|---:|---:|',
  ]
  for r in json.loads((OUT / 'benchmark.json').read_text()):
    lines.append(f"| {r['model']} | {r['median_ms']:.4f} | {r['p95_ms']:.4f} | {r['max_ms']:.4f} |")
  lines += [
    '',
    'Timing measures complete high-accel controller updates (2,000 measured frames after 200 warmup) on Mac Studio. A 5–10× hardware slowdown implies roughly 0.44–0.89 ms median, consistent with the requested ~2 ms target. This is not a Snapdragon deadline guarantee; no device benchmark was authorized or attempted.',
    '',
    'New source: tools/odyssey_sim/nnlc_smooth/{experiment.py,score.py,replay.py,benchmark.py,report.py,test_runtime.py,README.md}. Candidate JSONs are untracked inside the neural_network_data submodule. Hashes:',
  ]
  for name in ('S3', 'S4', 'S5'):
    p = MODEL_DIR / f'HONDA_ODYSSEY_WILLIAM_{name}.json'
    lines.append(f'- `{p.name}` SHA256 `{hashlib.sha256(p.read_bytes()).hexdigest()}`')
  metric = Path('tools/odyssey_sim/perception/metric.py')
  lines += [
    '',
    f'Perception source SHA256 `{hashlib.sha256(metric.read_bytes()).hexdigest()}`.',
    '',
    'Artifacts: scratchpad/nnlc_smooth/{generations.json,selection_scored.json,test.json,test_scored.json,selected.json,replay.json,benchmark.json} and all native NPZ traces; logs copied alongside them. The README contains reproduction commands and the explicit default-off model selection. All existing branch/untracked work is preserved; nothing staged.',
    '',
    '## Remaining risks and next concrete step',
    '',
    'The plant is identified at 10 Hz; high-frequency wheel/hand perception is not independently validated. No calibrated hand-torque response is available; no hands-on detection threshold for William is known. First-order plant motion and quantized feedback can suppress real EPS oscillation modes. Original selection angle conversion uses CarParams defaults; final test traces use per-frame steer ratio/stiffness for wheel-motion reconstruction. Public body dynamics and physical parameter uncertainty are sensitivity checks, not confidence bounds. Recorded plans and closed-loop historical data do not identify counterfactual planner/lane behaviour.',
    '',
    'The lead can package S3 and S5 with the bounded hook, then combine the inner-loop candidates with the wander/calib3 worker’s validated lane-loop slice and the low-speed/curves workers’ separate add-ons. Do not silently stack independent controller interventions: rerun the integrated candidate against stock with the same masks and complete wheel/body/hand coverage. Any vehicle rollout remains a separate approved lead action.',
    '',
    '## Plain-language summary',
    '',
    'The van-specific model follows the requested steering better in the calibrated simulator. Its extra electrical command reversals are not a sound reason to throw it away. A very short filter keeps that tracking gain and makes the command quieter; stronger-feedback versions follow straights still better. We have reviewable, default-off candidates and reproducible tests. We have not yet proved reduced real lane wandering, better parking-speed steering, earlier real curve entry or William-perceived hand smoothness. No vehicle settings changed.',
  ]
  (S / 'report_nnlc_smooth.md').write_text('\n'.join(lines) + '\n')
  print(S / 'report_nnlc_smooth.md')


if __name__ == '__main__':
  main()
