"""Reproducible Markdown report and chart data from completed sweep artifacts."""

import json
# ruff: noqa: E501 -- Markdown prose templates preserve paragraph boundaries.
import numpy as np
from tools.odyssey_sim.closedloop.data import SCRATCH
from tools.odyssey_sim.closedloop.controller import configs


def label(cfg):
  mode = 'NNLC' if cfg.nnlc else 'Stock'
  bits = [mode, f'{cfg.delay:.2f}s']
  if cfg.angle:
    bits.append('angle')
  if cfg.friction != 1:
    bits.append('friction×0.5')
  if cfg.threshold != 0.3:
    bits.append('threshold0.6')
  if cfg.kick:
    bits.append('kick0.03')
  return ' + '.join(bits)


def aggregate(rows):
  n = sum(r['n'] for r in rows)
  out = {k: sum(r[k] * r['n'] for r in rows) / n for k in ('lataccel', 'jerk', 'total', 'saturation_pct', 'flips_s', 'logged_lataccel')}
  out['n'] = n
  for key, count in (('entry_s', 'entries'), ('exit_s', 'exits')):
    total = sum(r[count] for r in rows)
    out[key] = sum(r[key] * r[count] for r in rows if r[key] is not None) / total if total else None
    out[count] = total
  return out


def main():
  cs = list(configs())
  names = {c.name: label(c) for c in cs}
  data = {p: json.loads((SCRATCH / f'closedloop_{p}.json').read_text()) for p in ('provisional', 'odyssey', 'public', 'tiny')}
  for p, rs in data.items():
    assert len(rs) == 350, (p, len(rs))
    assert all(r['nn_active'] > 0 if r['config'].startswith('nnlc') else r['nn_active'] == 0 for r in rs)
  summary = {p: {c.name: aggregate([r for r in rs if r['config'] == c.name]) for c in cs} for p, rs in data.items()}
  (SCRATCH / 'closedloop_summary.json').write_text(json.dumps(summary, indent=2))
  lines = [
    '# Odyssey closed-loop steering comparison',
    '',
    'Offline research only. No vehicle access, parameter deployment, tracked-file edits, commits or pushes.',
    '',
    '## Plain-language answer',
    '',
    'At ≥20 mph, NNLC 0.32 s has 8.4% lower cost than stock V0 in the William plant (447.85 versus 489.18), but is essentially tied in the public overlay (1554.62 versus 1554.99). NNLC 0.45 s raises William-plant reversals by 39% (0.67 to 0.93/s) without a robust tracking benefit. The fitted Odyssey plant and the generic comma plant disagree about what wins. This experiment identifies candidates for further validation; it does not establish the best controller for the real van. Low-speed angle steering cannot be judged reliably because neither available learned plant validates that regime.',
    '',
  ]
  lines += [
    '## Overall results',
    '',
    'Challenge cost is 50 × lateral-acceleration cost + jerk cost; lower is better. Figures below weight each route/window by eligible sample count. These are curve-heavy selected windows, not full-drive averages.',
    '',
    '|Configuration|Provisional total|William total|Public-overlay total|Generic tinyphysics total|',
    '|---|---:|---:|---:|---:|',
  ]
  for c in cs:
    lines.append('|' + names[c.name] + '|' + '|'.join(f'{summary[p][c.name]["total"]:.2f}' for p in data) + '|')
  lines += ['', '![Cost comparison](closedloop_chart.svg)', '', '## Ranked candidates and expected effect', '']
  base = cs[0].name
  valid_bands = ('20-35', '35-50', '50-200')
  valid = {p: {c.name: aggregate([r for r in rs if r['config'] == c.name and r['band'] in valid_bands]) for c in cs} for p, rs in data.items()}
  (SCRATCH / 'closedloop_summary_valid20plus.json').write_text(json.dumps(valid, indent=2))
  lines += ['**Primary ranking uses ≥20 mph only** (1,450 scored samples = 145 seconds per arm); overall totals above include invalid low-speed extrapolation. The fitted-model train routes include 1a7/18c/19b; 19c/1aa/1ab are route-held-out. Neither plant is calibrated for lane position.', '']
  best = sorted(valid['odyssey'], key=lambda k: valid['odyssey'][k]['total'])
  for rank, k in enumerate(best[:4], 1):
    pct = 100 * (1 - valid['odyssey'][k]['total'] / valid['odyssey'][base]['total'])
    lines.append(f'{rank}. **{names[k]}**: William ≥20 mph total {valid["odyssey"][k]["total"]:.2f}, {pct:.1f}% below stock. Public-overlay {valid["public"][k]["total"]:.2f}; provisional {valid["provisional"][k]["total"]:.2f}; generic {valid["tiny"][k]["total"]:.2f}.')
  lines += ['', '|≥20 mph configuration|William|Public-overlay|Provisional|Generic|', '|---|---:|---:|---:|---:|']
  for c in cs:
    lines.append('|' + names[c.name] + '|' + '|'.join(f'{valid[p][c.name]["total"]:.2f}' for p in ('odyssey','public','provisional','tiny')) + '|')
  lines += ['', 'Route-held-out ≥20 mph robustness:', '', '|Configuration|William|Public overlay|', '|---|---:|---:|']
  for c in cs:
    vals = [aggregate([r for r in data[p] if r['config']==c.name and r['band'] in valid_bands and r['route'].startswith(('000001aa','000001ab','0000019c'))])['total'] for p in ('odyssey','public')]
    lines.append(f'|{names[c.name]}|{vals[0]:.2f}|{vals[1]:.2f}|')
  lines += [
    '',
    'Recommendation: retain the current road configuration until the fitted model passes independent trajectory validation; keep 0.32 s as the preferred delay, and prioritize unmodified NNLC versus stock as the next robustness comparison. Avoid deploying the combined arm based on this ranking alone: interactions obscure what helped. The van’s driver takeovers, lateral path error and safety cannot be predicted by these acceleration-only costs. At ≥20 mph, the 0.03 kick saves only about 0.4–0.5% versus NNLC alone on the William/public plants while raising command reversals; it is not a compelling change from this evidence. Stock V0 is delay-invariant in these runs; NNLC 0.45 s increases reversals and has no robust tracking advantage.',
    '',
    'Delay changes: 0.45 s changes both LaC history alignment and NNLC future interpolation. The logged upstream plan remains fixed, so this is not a complete replay of changing steerActuatorDelay in the running planner.',
    '',
    'Friction softening can reduce oscillation but may delay breaking static friction; a 0.03 kick can improve deadband response while increasing chatter or near-zero bias. NNLC has friction learned into its model: halving the explicit torque-parameter friction does not halve learned NN friction. Angle mode uses the lead’s real controller-independent OdysseySteer.apply after V0/NNLC, plus the V0 odyssey_kp_interp wiring (gain cap8 when angle enabled). The deadband kick also executes in this shared post-processing path. The prior V1-only wiring did not execute on the recorded drives; these arms predict the newly corrected wiring, rather than historical feature performance.',
    '',
    '## Harness, scope and validation',
    '',
    '- Uses the real V0 LatControlTorque, sunnypilot extension, Odyssey NNLC JSON, VehicleModel and real logged modelV2 arrays. See tools/odyssey_sim/closedloop/README.md for exact execution commands and conventions.',
    '- All six requested drives are decoded from local rlog FILE paths. At 10 Hz the CSV preserves speed, longitudinal acceleration, roll and planned lateral acceleration; NPZ adds future acceleration/orientation arrays. The actual controller curvature input is carControl.actuators.curvature. The controlsState desiredCurvature is retained separately; their eligible route RMS acceleration difference is 0.0031–0.0096 m/s².',
    '- 25 selected 30-second windows (750 seconds of context) per plant/configuration, ten seconds discarded, then band/latActive/!steeringPressed masking. Each window has at least three eligible seconds. The same window can be selected for adjacent bands; scores never count out-of-band samples. This deliberately stresses curves and overweights difficult episodes.',
    '- Native plants execute actual controller and dynamics at 100 Hz with 10 Hz logged exogenous inputs held. Tinyphysics imports comma’s actual TinyPhysicsSimulator/Model and uses 10 Hz physics with ten native controller calls per observation. It has no interpolated 100 Hz motion feedback, so its jerk/history response is a baseline approximation.',
    '- Tinyphysics uses its original tokenizer, stochastic sampling, history, warmup and ±0.5 m/s²-per-step cap. Seed is fixed per route/window and shared across arms. One seed per window is insufficient for confidence intervals.',
    '- The provisional plant assumes gain1.47, deadband0.03, Coulomb-like friction0.145, offset−0.055 and lag0.4s. These are priors, not fitted dynamics. Gain, friction, hysteresis and lag uncertainty can reverse rankings.',
    '- Corrected field names: live roll/ratio/offset from vehicleParameters (roll std 0.0177–0.0264 rad across routes); torque parameters are lateralTorqueParameters and delay lateralDelay in validation. The controller comparison deliberately fixes learned torque params at 1.47/0.145/−0.055 and delays at 0.32/0.45. calibrated_pose is None because livePose is unavailable, so NNLC pitch correction is not reconstructed. Worker 2’s William roll gain is a prior because its fit extractor missed this field; applying that fit with live roll is a material model uncertainty.',
    '- Worker 2 schema-v1 step() plant is used with a frozen parameter snapshot. Feedback/cost uses steering_accel (wheel-derived acceleration, matching torqueState), rather than separately lagged inertial lat_accel. Real VehicleModel provides the inverse wheel-angle measurement. Worker plant’s own bicycle angle output is not fed directly into a different vehicle model.',
    '- No CAN torque rate limiter, Honda safety/driver torque limiter, driver input, sensor quantization, lateral planner response or lane geometry feedback. Existing inactive or driver-pressed context is still simulated with control active; those samples are excluded from costs.',
    '- All fourteen construction settings are verified by assertions and check.py; positive torque increases plant acceleration; real VehicleModel inverse feedback error <1e-9 m/s²; cost identity checks pass. Params writes are blocking and every run has an isolated simclosed-prefixed namespace.',
    '',
  ]
  val = json.loads((SCRATCH / 'closedloop_validation.json').read_text())
  lines += [
    'Logged-motion validation: ' + json.dumps(val),
    '',
    'The matching v0 NNLC replay closely reproduces logged commands. The separate v1 diagnostic differs from V0 in the logs (RMSE ~0.0206 normalized torque); all final sweeps use V0, which matches logged v0 with RMSE ~0.00020. This validates data/sign/instantiation, not closed-loop plant fidelity.',
    '',
    'Stock-controller simulated versus recorded tracking on identical selected samples:',
    '',
    '|Speed mph|Recorded accel cost|Stock provisional|Stock William|Stock public|Stock tinyphysics|',
    '|---|---:|---:|---:|---:|---:|',
  ]
  bands = ['0-10', '10-20', '20-35', '35-50', '50-200']
  for b in bands:
    rs = [r for r in data['odyssey'] if r['config'] == base and r['band'] == b]
    a = aggregate(rs)
    costs = [aggregate([r for r in data[p] if r['config'] == base and r['band'] == b])['lataccel'] for p in data]
    lines.append(f'|{b.replace("50-200", "50+")}|{a["logged_lataccel"]:.3f}|' + '|'.join(f'{x:.3f}' for x in costs) + '|')
  lines += [
    '',
    '**The stock simulated tracking costs do not reproduce recorded tracking costs consistently.** These are fixed learned-parameter/V0 runs, whereas the logs contain dynamic parameters, NNLC/v0 and driver/system transitions. Even after accounting for those differences, this is a failed criterion for claiming validated van prediction. Plant fit validation and held-out results in report_plant.md bound all interpretation.',
    '',
    '## Per-plant and speed-band metrics',
    '',
    'L = mean squared accel error ×100; J = mean squared 10 Hz acceleration derivative ×100; T=50L+J. Sat is |torque|≥0.99 command percentage. Physical Sat/flips use 100 Hz commands; generic uses applied 10 Hz commands. Flips require >0.01 torque change; adjacent samples must both be eligible. Entry/exit are sample-count-weighted means of route medians in seconds, with matched event counts in parentheses. NA means no matched event, not zero lag. Positive is late, negative early. Crossing threshold |accel|=0.3 m/s²; nearest crossing within ±2 seconds is descriptive and may mismatch oscillatory events. Aggregated J is eligible-sample-weighted rather than pooled-adjacent-pair weighted.',
    '',
  ]
  for p, rs in data.items():
    lines += [f'### {p}', '']
    if p in ('odyssey','public'):
      lines += [
        '**0–10 mph is outside the plant’s intended range; speed is clamped at 5 m/s. Those rows are extrapolation diagnostics only. 10–20 mph includes some samples below that limit.**',
        '',
      ]
    for b in bands:
      lines += [
        f'#### {b.replace("50-200", "50+")} mph',
        '',
        '|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|',
      ]
      for c in cs:
        a = aggregate([r for r in rs if r['config'] == c.name and r['band'] == b])

        def fmt(k, n, a=a):
          return 'NA' if a[k] is None else f'{a[k]:.2f} ({a[n]})'

        lines.append(
          f'|{names[c.name]}|{a["lataccel"]:.3f}|{a["jerk"]:.2f}|{a["total"]:.2f}|{a["saturation_pct"]:.2f}|{a["flips_s"]:.2f}|{fmt("entry_s", "entries")}|{fmt("exit_s", "exits")}|{a["n"]}|'
        )
      lines += ['']
  lines += [
    '## Evidence and handoff',
    '',
    '- Production provenance: closedloop_provenance.json records source hashes and branch/head. This worker only writes new closedloop files and scratch artifacts; concurrent tracked changes/commits belong to other workers. Stock V0 delay invariance is exact in the William sweep (maximum cost difference 0).',
    '- Raw per-window results: closedloop_{provisional,odyssey,public,tiny}.json; commands/prediction traces in matching *_traces.npz. frozen worker plant: closedloop_{odyssey,public}_plant_snapshot.json.',
    '- Full converted logs: closedloop_data/*.csv and *.npz. Controller/metric checks: closedloop_checks.json. Replay validation: closedloop_validation.json.',
    '- Reproduction scripts: tools/odyssey_sim/closedloop/{data,controller,rollout,worker_plant,check,validate,report}.py and README.md. No existing tracked files modified by this worker; no commit or push.',
    '- Self-review corrected asynchronous Params initialization and checked the V0 source baseline friction threshold (0.3). All results shown use blocking writes and source-correct V0 baseline, with final production OdysseySteer/KP-cap wiring. No additional subagents were spawned because this task explicitly assigned a single research worker.',
    '- Exact next validation: run matched logged command sequences through the frozen plant on independent segments; quantify 1/3/5-second wheel-angle-derived acceleration error per speed band, then rerun this harness over multiple windows/seeds and bounded plant parameter perturbations before selecting a road-test setting.',
    '',
  ]
  (SCRATCH / 'report_closedloop.md').write_text('\n'.join(lines))
  # Standalone vector chart for the report, no additional plotting dependencies.
  colors = ['#5678c4', '#bc5c38', '#8756b0', '#479e86']
  svg = [
    '<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="970"><rect width="100%" height="100%" fill="white"/>',
    '<text x="10" y="25" font-size="18">Curve-heavy challenge cost: plant rankings disagree (log scale)</text>',
  ]
  maxv = max(summary[p][c.name]['total'] for p in data for c in cs)
  for j, (p, color) in enumerate(zip(data, colors, strict=True)):
    svg.append(f'<text x="{330 + j * 160}" y="50" fill="{color}">{p}</text>')
  for i, c in enumerate(cs):
    y = 75 + i * 62
    svg.append(f'<text x="5" y="{y + 18}" font-size="12">{names[c.name]}</text>')
    for j, (p, color) in enumerate(zip(data, colors, strict=True)):
      value = summary[p][c.name]['total']
      w = 550 * np.log10(1 + value) / np.log10(1 + maxv)
      svg.append(
        f'<rect x="340" y="{y + j * 12}" width="{w:.1f}" height="10" fill="{color}"/><text x="{345 + w:.1f}" y="{y + j * 12 + 9}" font-size="10">{value:.1f}</text>'
      )
  svg.append('</svg>')
  (SCRATCH / 'closedloop_chart.svg').write_text(''.join(svg))
  print('Report complete', [(p, min(summary[p], key=lambda k: summary[p][k]['total'])) for p in data])


if __name__ == '__main__':
  main()
