"""Write the requested evidence receipt from final experiment outputs."""

import hashlib
import json
import numpy as np
from tools.odyssey_sim.nnlc_train import SCRATCH as S, MODEL_DIR


def main():
  manifest = json.loads((S / 'nnlc_data_manifest.json').read_text())
  scores = json.loads((S / 'nnlc_scores.json').read_text())
  closed = json.loads((S / 'nnlc_closedloop.json').read_text())
  replay = json.loads((S / 'nnlc_replay.json').read_text())
  benchmark = json.loads((S / 'nnlc_benchmark.json').read_text())
  lines = [
    '# Van-specific NNLC experiment — final implementation receipt',
    '',
    (
      '**Recommendation: retain stock V0 + HONDA_ODYSSEY NNLC as the default. V1 is an opt-in research '
      + 'candidate; do not deploy V2.** Both trained models reduce held-out inverse torque error, but closed-loop '
      + 'jerk and reversal regressions prevent a claim of better driving. No device was accessed, no branch '
      + 'switched, no commit or push made.'
    ),
    '',
    '## Data and training',
    '',
    (
      'Public source: https://github.com/commaai/comma-steering-control. Reused the plant worker’s deterministic'
      + ' every-tenth-segment cache: 1,499 of 14,987 sixty-second CSVs (~25 hours). After filtering and '
      + 'every-other-frame sampling: 333,232 examples. Segment identities remain intact; random frames are used '
      + 'only for optimizer minibatches, never to split training/test. Community vehicle identity is unavailable.'
    ),
    '',
    'William’s EPS is `39990-THR-A020` on all six included routes. Public sampled firmware counts: '
    + str(manifest['firmware_segments'])
    + (
      '. A020 samples carry weight 3, other firmware weight 1. Small |torque|<0.1 receives another factor 3. '
      + 'Both candidates pretrain on public data and adapt to William’s van. Public data was not expanded to all '
      + '250 hours: this already supplies >20x as many examples as the van, whose measured gain differs '
      + 'substantially; model selection on a complete owner route stops pretraining/adaptation when it ceases '
      + 'helping.'
    ),
    '',
    '| Owner route | Role | Examples |',
    '|---|---|---:|',
  ]
  for route, d in manifest['own'].items():
    role = 'final test' if route.startswith(('000001aa', '000001ab')) else 'weight selection' if route.startswith('000001a7') else 'adaptation'
    lines.append(f'| {route} | {role} | {d["n"]} |')
  lines += [
    '',
    (
      'The available closed-loop caches cover 18c,19b,19c,a7,aa,ab. a3 and other routes were not added. Every '
      + 'owner response is recomputed from steering angle minus live angle offset, using per-frame '
      + 'steerRatio/stiffness and vehicleParameters.roll in VehicleModel. Labels are applied CAN STEER_TORQUE/4096'
      + ' interpolated from the plant cache; this sign matches positive openpilot curvature. Public labels are '
      + 'negative steerFiltered. An initial exploratory fit used negative logged PID output; it was superseded '
      + 'after observing 0.01–0.05 mean command differences from applied CAN. All tables here use the corrected '
      + 'applied-CAN fit.'
    ),
    '',
    (
      'Both models: deterministic seed 20261109, CPU torch training in separate scratch nnlc-venv, Adam lr .002,'
      + ' 4096-ish minibatches, up to 180,000 weighted draws per epoch, 35 public epochs then 60 owner epochs, '
      + 'select the best weighted validation MSE on a7 throughout. Loss includes a penalty for an insufficient '
      + 'positive restoring gradient under simultaneous acceleration-input increases. Checkpoint selection never '
      + 'sees aa/ab. Hidden layers use sigmoid; output is linear. V1 18→64→32→1 (3,329 parameters), V2 26→64→32→1 '
      + '(3,841 parameters). No torch enters the device runtime.'
    ),
    '',
    (
      'V1 preserves the original 18-input schema. V2 adds angle/rate, current acceleration, acceleration '
      + '0.1/0.3/0.5 seconds ago, and applied torque 0.3/0.5 seconds ago. Its offline closed-loop adapter '
      + 'substitutes pre-NNLC controller-output history for unavailable applied CAN history. This approximation and '
      + 'autoregressive persistence are limitations, not evidence of a learned EPS law.'
    ),
    '',
    (
      'The inverse task uses achieved steering-angle acceleration 0.32 s after command time, plus its '
      + '-0.3/-0.2/-0.1/+0.3/+0.6/+1.0/+1.5 context. Future achieved signals are permissible inverse '
      + 'identification targets but are NOT available to the deployed controller. Runtime NNLC substitutes the '
      + 'plan and desired history. NNLC’s third input is actually its friction proxy, while the identification '
      + 'task uses acceleration derivative. Setpoint/measurement calls additionally repeat the instantaneous '
      + 'acceleration with LOW_SPEED curvature augmentation. These distribution differences are material. The '
      + 'fitted 0.87 s time constant is not a pure delay; it was not substituted for the 0.32 s alignment.'
    ),
    '',
    '## Held-out inverse torque prediction',
    '',
    (
      'Filtering excludes |applied torque| >= .98, speed <=1 m/s and acceleration magnitude >=4 m/s²; it '
      + 'requires active/no-driver intervention throughout each context window. Thus the error tables '
      + 'characterize nonsaturated inverse behavior. Only 52 test examples are below 10 mph; saturation '
      + 'performance is not established.'
    ),
    '',
    (
      'Torque is normalized; MAE 0.05 means five percent of full command. Metrics below pool aa/ab by sample '
      + 'count. No held-out frames were used to tune weights. All results are against applied CAN torque; they are'
      + ' not trajectory-following metrics.'
    ),
    '',
    '| Band | n | Stock MAE / RMSE | V1 MAE / RMSE | V2 MAE / RMSE |',
    '|---|---:|---:|---:|---:|',
  ]
  for band in ['all', 'small_torque', 'large_torque', '0-10mph', '10-20mph', '20-35mph', '35-50mph', '50-999mph']:
    cells = []
    for model in ['stock', 'v1', 'v2']:
      rs = [r for r in scores if r['band'] == band and r['model'] == model and r['n']]
      n = sum(r['n'] for r in rs)
      mae = sum(r['mae'] * r['n'] for r in rs) / n if n else np.nan
      rmse = np.sqrt(sum(r['rmse'] ** 2 * r['n'] for r in rs) / n) if n else np.nan
      cells.append(f'{mae:.4f} / {rmse:.4f}' if n else 'no coverage')
    lines.append(f'| {band.replace("50-999mph", "50+ mph")} | {n} | ' + ' | '.join(cells) + ' |')
  lines += ['', '| Route / slice | Stock MAE | V1 MAE | V2 MAE |', '|---|---:|---:|---:|']
  for route in ['000001aa--ba236ee054', '000001ab--9191c0d55d']:
    for band in ['all', 'small_torque']:
      cells = [next(r for r in scores if r['route'] == route and r['band'] == band and r['model'] == m)['mae'] for m in ['stock', 'v1', 'v2']]
      lines.append(f'| {route[:8]} / {band} | ' + ' | '.join(f'{v:.4f}' for v in cells) + ' |')
  lines += [
    '',
    '## Closed loop on worker 2’s fitted plant',
    '',
    (
      'Reused worker 3’s windows, Controller interface, 100 Hz updates, plant adapter and Controls Challenge '
      + 'cost convention. Explicitly replaced its V1 core with the van’s V0 class for ALL arms. V1 uses the actual'
      + ' bounded runtime evaluator. No low-speed-angle/deadband post-step was enabled. Both trained models pass '
      + 'through the existing NNLC friction override and torque-space PID. Fixed .32 s controller delay and logged'
      + ' modelV2 plans; learned plant gain/friction/lag. aa/ab are excluded from plant fitting. Costs use '
      + '50×lataccel_cost + jerk_cost; lower is better.'
    ),
    '',
    (
      'Cells are **cost / acceleration jerk-cost / native 100 Hz torque reversals per second**. Each row is one '
      + 'worker-selected 30 s maximum-curvature window with 10 s warmup and a speed-band mask. Some bands have '
      + 'only a few scored seconds. No Honda actuator rate limiter or closed planner/lane feedback is simulated; '
      + 'this is a rejection screen, not proof of a road win. Plant validity begins at 5 m/s (~11 mph), so 0–10 '
      + 'mph rows are explicitly extrapolations and cannot support low-speed claims.'
    ),
    '',
    '| Route / mph | scored n (10 Hz) | Stock | V1 | V2 |',
    '|---|---:|---:|---:|---:|',
  ]
  for route, band in dict.fromkeys((r['route'], r['band']) for r in closed):
    rs = [next(r for r in closed if r['route'] == route and r['band'] == band and r['model'] == m) for m in ['stock', 'v1', 'v2']]
    lines.append(f'| {route[:8]} / {band} | {rs[0]["n"]} | ' + ' | '.join(f'{r["total"]:.2f} / {r["jerk"]:.2f} / {r["native_flips_s"]:.2f}' for r in rs) + ' |')
  lines += [
    '',
    (
      'V2 does not consistently beat V1, and both can increase jerk/reversals against stock. It is therefore '
      + 'retained only as a reproducible offline research artifact; there is no extended-model runtime toggle to '
      + 'accidentally enable it. The generic tinyphysics ONNX plant was not rerun for these candidates '
      + '(onnxruntime is absent in the repo venv); the Odyssey-specific fitted plant was run. Worker 3’s existing '
      + 'generic comparison is separate evidence, not a result for these candidates.'
    ),
    '',
    '## Logged-motion replay',
    '',
    (
      'Reused worker 1’s validated replay adapter, V0, original timing/lag and vehicleParameters. Bounded V1 vs '
      + 'stock on the first 20,000 control frames of each final-test route. Params writes are synchronous in the '
      + 'isolated replay namespace. MAE is agreement with logged PID output, not evidence of better steering; '
      + 'motion remains logged/open-loop.'
    ),
    '',
    '| Route / mph | n | Stock command MAE / flips per s | V1 command MAE / flips per s |',
    '|---|---:|---:|---:|',
  ]
  for route, band in dict.fromkeys((r['route'], r['band']) for r in replay):
    rs = [next(r for r in replay if r['route'] == route and r['band'] == band and r['model'] == m) for m in ['stock', 'v1']]
    lines.append(f'| {route[:8]} / {band} | {rs[0]["n"]} | ' + ' | '.join(f'{r["command_mae"]:.4f} / {r["flips_s"]:.2f}' for r in rs) + ' |')
  lines += [
    '',
    '## Timing',
    '',
    (
      'Full controller updates at high acceleration (including error blending and up to five model evaluations),'
      + ' 2000 iterations after warmup, Mac Studio; not on-device measurements.'
    ),
    '',
    '| Model | Mac median / p95 ms | 3X estimated median ms (5–10× slower) |',
    '|---|---:|---:|',
  ]
  for name, d in benchmark.items():
    est = d['estimated_3x_median_ms_5to10x']
    lines.append(f'| {name} | {d["mac_median_ms"]:.4f} / {d["mac_p95_ms"]:.4f} | {est[0]:.3f}–{est[1]:.3f} |')
  lines += [
    '',
    (
      'This fits the requested ~2 ms estimate, including the complete controller update. It is not a certified '
      + 'worst-case deadline on Snapdragon hardware. No device benchmark was performed.'
    ),
    '',
    '## Implementation, checks, and enabling',
    '',
    (
      (
        'Uncommitted changes include an ODYSSEY.md link to the experiment guide. In the root checkout: '
        + '`openpilot/common/params_keys.h` (persistent/backed-up string '
      )
      + 'NnlcModelOverride), `nnlc/helpers.py` (validated selection, exclude candidate names from automatic '
      + 'matching), `nnlc/nnlc.py` (shared extension selection for V0 and V1), new `nnlc/safe_model.py` (bounded '
      + 'experimental inference and sanitized-stock fallback), new `nnlc/tests/test_model_override.py`, and the '
      + 'new `tools/odyssey_sim/nnlc_train/` scripts/README. Candidate JSONs are untracked inside the '
      + 'neural_network_data submodule, so the parent status shows that submodule dirty; a lead must package those'
      + ' artifacts explicitly. Existing stock JSON is unchanged.'
    ),
    '',
    'Candidate hashes:',
  ]
  for name in ['HONDA_ODYSSEY_WILLIAM_V1.json', 'HONDA_ODYSSEY_WILLIAM_V2.json']:
    lines.append(f'- `{name}` SHA256 `{hashlib.sha256((MODEL_DIR / name).read_bytes()).hexdigest()}`')
  lines += [
    '',
    (
      'Checks: params rebuilt locally with the requested scons target (build log nnlc_params_build.log); all 35 '
      + 'NNLC tests pass (nnlc_tests.log); targeted ruff passes. Tests cover unchanged default matching, '
      + 'absent/missing/path-traversal/unsupported overrides, real Params selection, finite bounded outputs '
      + 'including NaN/Inf fallback, monotonic coordinated acceleration sweeps across speed/roll, and selection '
      + 'through both controller classes. The override tests were observed failing before implementation, then '
      + 'passing. No subagents were spawned because this task is itself an implementation-subagent assignment; '
      + 'implementation self-review covered data/signs, default-off isolation, then runtime bounds and '
      + 'reproducibility.'
    ),
    '',
    (
      'To stage V1 on a reviewed build, while offroad set `NnlcModelOverride` to '
      + '`HONDA_ODYSSEY_WILLIAM_V1.json`, keep `NeuralNetworkLateralControl=1`, disable `NnlcAB` for a fixed-model'
      + ' comparison, then restart controls/reboot. The new param is read at controller construction. Remove the '
      + 'override and restart to restore stock. See `tools/odyssey_sim/nnlc_train/README.md` for exact Params '
      + 'commands. No configuration was written to the van. V2 cannot be selected by this override (26 inputs are '
      + 'rejected).'
    ),
    '',
    (
      'Artifacts: nnlc_data_manifest.json, nnlc_train.log, nnlc_scores.json, nnlc_closedloop.json, '
      + 'nnlc_replay.json, nnlc_benchmark.json, nnlc_tests.log in this scratch directory. Public-only checkpoints '
      + 'also remain here, not in the shipped model directory.'
    ),
    '',
    '## What William should take away',
    '',
    (
      'The van-specific models learned the small-command region better than the old community model. That alone '
      + 'does not solve the van: the controller can still chatter, saturation obscures inverse identification, '
      + 'future desired motion differs from future achieved motion, and the planner’s slow curve-exit request '
      + 'remains upstream. V1 is ready for lead review as an experiment; neither model has earned default '
      + 'deployment. The next concrete step is to fit a causal controller objective that penalizes command '
      + 'reversals and uses applied actuator history consistently, then verify through the validated V0 replay and'
      + ' rate-limited closed loop before any road comparison. Full 250-hour pretraining or a larger network is '
      + 'secondary to that objective/runtime alignment.'
    ),
  ]
  (S / 'report_van_nnlc.md').write_text('\n'.join(lines) + '\n')


if __name__ == '__main__':
  main()
