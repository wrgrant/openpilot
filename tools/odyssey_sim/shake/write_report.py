# ruff: noqa: E501
"""Build research report from frozen results; no extra parameter fitting."""
import hashlib
import json
from pathlib import Path
import numpy as np
from tools.odyssey_sim.shake.analyze import OUT


def main():
  g1 = json.loads((OUT/'gate1.json').read_text())
  g2 = json.loads((OUT/'gate2.json').read_text())
  fit = json.loads((OUT/'fit.json').read_text())
  diagnostic = json.loads((OUT/'diagnostics.json').read_text())
  ladder = json.loads((OUT/'ladder_diagnostic.json').read_text())
  model = ['stock NNLC', 'S3', 'S5']
  text = '''# Shake research: rejected physical plant, 9 November 2026

**No: this simulator does not yet reproduce what you felt well enough to tune steering.** It captures much of S5's fast motion when fed the recorded command, but fails the held-out gain/phase limits for every controller and speed band. In closed loop it produces only **65% of S5's recorded body-motion RMS and 59% of its command crossings**. Both validation gates fail. There is no justified quiet-bump candidate or locked road forecast.

Your US-169 report remains the reference: typical push **±20 N (±0.25 m/s²)**, worst **±40–50 N (±0.50–0.625 m/s²)**, about **1.3 oscillations per second** and **2.7 direction changes per second**, with lateral jerk RMS 1.1 m/s³. Earlier drives were **±7–9 N (±0.0875–0.1125 m/s²)** around **0.3 oscillations per second**. These typical peaks are different statistics from the RMS quantities below.

The useful result is a reproducible failure boundary. A physically structured model comes closer than the old scalar plant on a training shake window, but motor lag, tyre relaxation and column dynamics remain confounded. Adding a roll oscillator does **not** establish that roll caused the yaw shake. The extracted orientation cannot identify a credible damped roll mode. The next useful work is a deliberately independent excitation/measurement session, followed by a fresh held-out run—not tuning another controller in this rejected simulator.

## Scope and evidence

Work stayed on `odyssey-sim`; no commits, pushes, branch changes, device or van access. Code/results/plots are in `tools/odyssey_sim/shake/`; augmented log caches and dependencies are in the permitted session `scratchpad/shake/`. This report is the explicitly requested exception to the folder-only write rule. The plot links below point into `../shake/`, respecting that restriction rather than writing other report-side artifacts.

There is no root ARCHITECTURE.md or repository check command in this checkout; ODYSSEY.md supplied the map. No shared runtime files were edited. Runtime APIs have moved since the older adapters: the local Params.get lacks `encoding`, while current NNLC calls it; the scratch-only adapter supplies that compatibility. Current OdysseySteer also needs blinker fields absent from the old adapter. These were fixed locally in the research adapter, not in runtime code.

Read the result artifacts as rejected-model diagnostics, not road predictions. `fit.json` was written before opening 1ad/1ae for evaluation. Neither held-out route entered optimization. Evaluator frequency bins were subsequently corrected to the exact requested frequencies, without changing parameters. Their use for this diagnosis now means they cannot serve as untouched holdouts for another generation informed by these failures.

Data: existing six 100-per-second tables; all 1ac/1ad/1ae rlogs supplemented with CAN motor magnitude and actual CAN command, raw accelerometer/gyro vectors, orientation, vehicle/torque/delay calibration, and model plans. Raw inertial vectors are retained in device coordinates; sensor-source identity/extrinsic transformations are not yet carried through to an independently validated vehicle-frame acceleration. They are **not** silently treated as seat acceleration. OrientationNED roll is only a diagnostic proxy. No video inference is necessary for the fixed-plan gates.

## Model and identification

The simulator has column angle/velocity, lateral velocity, yaw rate, front/rear tyre forces, roll angle/rate, EPS lag state, command slew state and optional Dahl presliding state. It runs at 1 ms internally with command updates 100 times per second. Positive command and angle inside the model mean positive curvature: logged actuator torque and wheel angle are negated; logged yaw is not.

Equations (all mechanics in SI):

- EPS command slews at 3 full-scale units/s. Motor target is an interpolated saturating deadband map, followed by `tau_m * Tdot + T = map(u)`. Motor raw units are assigned an **assumed** 0.05 N·m/raw conversion.
- Column: `J*theta_ddot = T - B*theta_dot - friction - trail*F_front/steering_ratio`. Coulomb stiction holds exactly at zero until net torque exceeds breakaway. The alternative Dahl model uses `z_dot = theta_dot - abs(theta_dot)*z/d`, `friction = Fc*z/d`; this permits presliding rather than a smoothed sign function. Aligning torque grows with front tyre force and hence with speed for a given steering angle. It is not an independently fitted arbitrary speed-gain table.
- Single track: `Ff_target = Cf*(theta/ratio - (vy+a*r)/v)` and `Fr_target = Cr*(-(vy-b*r)/v)`, each with relaxation time `length/v`; `m*(vy_dot+v*r)=Ff+Fr`; `Iz*r_dot=a*Ff-b*Fr`.
- Roll: `phi_ddot + 2*zeta*wn*phi_dot + wn²*phi = (ms*h/Iroll)*ay`. This is a one-way, small-angle sprung-roll extension, without roll-steer or lateral load-transfer tyre nonlinearities. Its assumed natural mode is 1.3 oscillations per second; it is not a measured van resonance.
- Gate body signal is `v*r`, as required and validated by prior yaw work. Actual CG acceleration is `vy_dot+v*r`. These differ during transients. For the chosen positive roll convention, a rigid point height `h_point` above CG has transverse acceleration `ay-h_point*phi_ddot`, and specific-force component `ay-h_point*phi_ddot-g*phi`. The diagnostic seat uses 0.5 m. Head motion additionally includes neck/seat compliance; the device additionally needs its full 3-D mounting rotation, longitudinal lever arm/yaw acceleration, gravity removal and time alignment. A windshield measurement is not a head/seat measurement. Neither one is validated here.

The bicycle balance equations follow [MathWorks' vehicle-body model documentation](https://www.mathworks.com/help/driving/ref/bicyclemodel.html). Dynamic presliding friction is motivated by [Canudas de Wit et al., 1995](https://lup.lub.lu.se/search/ws/files/6363840/8498924.pdf), although the implemented alternative is the simpler Dahl law, not a claimed full LuGre implementation. A coupled roll-model reference is [Solmaz et al., Vehicle System Dynamics](https://people.isy.liu.se/fs/frisk/tmp/iQDrive_Articles/Solmaz_2008.pdf).

### Parameter values and limits

[Honda's 2019 US specifications](https://hondanews.com/en-US/releases/release-3eff8d4ea13b4a169396647f4e298ea0-2019-honda-odyssey-specifications-features) give Elite curb mass 4593 lb (2083.35 kg), wheelbase 118.1 in (2.99974 m), front/rear tracks 67.3/67.2 in (1.7094/1.7069 m), nominal steering ratio 14.35:1, front weight share 55%, and **235/55 R19 101H** tyres. These are published configuration values, not measurements of this loaded van. The brief says 2019 Elite while older repo text says 2018: confirm the model year/trim. Regional brochures have different tyre specifications; the US source is used here.

| Parameter | Value used | Identification and uncertainty |
|---|---:|---|
| Total mass | 2163.35 kg | Published curb + assumed 80 kg driver only; occupants/cargo/fuel unknown; not fitted |
| CG to axles, front/rear | 1.35 / 1.65 m | Inferred from published 55/45 curb distribution; load distribution unknown |
| Steering ratio | 14.35 | Published nominal mechanical ratio; learned compliance ratio is different and remains a controller input |
| Sprung mass | 1900 kg | Assumed; not measured |
| Roll-centre height / sprung CG height | illustrative 0.25 / 0.75 m above ground | Assumed geometry; neither identifiable here; only their difference and inertia ratio enter the roll response |
| Roll inertia | illustrative 1357 kg·m² | Assumed from ms × 0.5 m / 0.7; not measured |
| Net restoring roll stiffness | about 90,550 N·m/rad | Assumed from inertia and 1.3 oscillations/s; physical suspension stiffness adds ms*g*h (about 9,320 N·m/rad) |
| Roll damping | about 6650 N·m·s/rad | Assumed damping ratio 0.3; unvalidated |
| Motor raw-to-column torque | 0.05 N·m/raw | Gauge assumption, not a torque calibration; absolute J/B/Fc/trail scale is not identified |
| EPS deadband / saturation | 0.028 / 75 raw | Earlier measured map, fixed in gate plant; new motor diagnostic does not validate it dynamically |
'''
  names = [('Column inertia', 'kg·m²'), ('Column viscous damping', 'N·m·s/rad'), ('Column breakaway', 'N·m'),
           ('Effective aligning trail', 'm'), ('Front axle cornering stiffness', 'N/rad'), ('Rear axle cornering stiffness', 'N/rad'),
           ('Yaw inertia', 'kg·m²'), ('Aggregate fitted motor lag', 's'), ('Tyre relaxation length', 'm')]
  for j, (name, unit) in enumerate(names):
    vals = [r['parameters'][j] for r in fit['trials']]
    text += f"| {name} | {fit['parameters'][j]:.5g} {unit} | Fitted; structural alternatives {min(vals):.5g}–{max(vals):.5g}; optimization bounds {fit['parameter_bounds'][0][j]}–{fit['parameter_bounds'][1][j]}; not a confidence interval |\n"
  text += '''| Dahl presliding displacement | 0.003 rad | Assumed candidate; Coulomb alternative uses zero |
| Roll mode / damping / drive | 1.3 oscillations/s / 0.3 / 0.7 m⁻¹ | Assumed in gate plant; roll-proxy identification rejected |
| Seat / head / device height above CG | 0.5 m / unknown / unknown | Seat is illustrative only; head and device need measurement |

The model alternatives have nearly identical scaled training MSE (Coulomb 0.34932; Dahl 0.34904), while column inertia changes from 0.039 to 0.216 kg·m². Rear cornering stiffness hits 300,000 N/rad and aggregate motor lag hits 0.25 s. This is confounding, not precise parameter recovery. No defensible 95% parameter intervals can be reported from this exercise. The listed structural spread and optimization bounds are not statistical uncertainty. A multistart/profile/bootstrap identification with independent excitation is still required before physical estimates are accepted.

The separately fitted **measured motor** magnitude response on 1ac (`eps.json`) gives lag **0.02395 s**, map scale **0.9418**, and **11.04 raw-unit RMS residual**—large against a nominal 75-unit cap. The fitted curve ends at 70.64 raw units. The deadband was fixed, not re-identified. Hands can add motor assist, the signal has no sign, and the magnitude at reversal is not a signed torque measurement. This diagnostic is evidence against interpreting the plant's 0.25 s lag as EPS electrical delay. It was not quietly replaced after looking at the holdout. Separate signed motor/rack torque calibration and reversal excitation are required.

The roll-proxy regression on training orientation gives **negative damping −0.540 s⁻¹** unconstrained. Positive-constrained fitting pins damping at its floor (0.010 s⁻¹), with natural rate 0.777 oscillations/s, damping ratio 0.0010, 55% derivative variance explained, and coherence only 0.505 at 1.3 oscillations/s. This is not a credible suspension identification. Road bank, processed orientation, uncalibrated sensor axes and proxy lateral acceleration remain mixed. No claim that the van's roll mode is at 1.3 oscillations/s follows from this data processing.

### Training and scoring protocol

Train pool: 19 valid 20 s windows from 1ac, plus 53 from 1a7/1aa/1ab. Fit uses 22 deterministically selected windows, balanced up to four per controller/speed cell plus six earlier windows. These are ordinary closed-loop driving records, not independent randomized plant excitation. Fit nine log-positive parameters against bandpassed wheel/yaw trajectories with robust least squares; try exact Coulomb and Dahl, choose on training residual only. Maximum 65 function evaluations per variant; the Coulomb run used 25, Dahl 55. Local minima and boundary solutions remain limitations.

Holdouts: 17 continuous 20 s windows, 5 stock, 6 S3, 6 S5 (1ad+1ae). Split at segment, timestamp discontinuity, speed-band, active/blinker, and 180 s experiment boundaries. Exclude each block's first 30 s; require steering active, no blinkers, above 25 mph. **Do not exclude steeringPressed**: it is frequently self-motion on straights. This also means true driver interventions can contaminate some curves; rows retain driver torque and press data for future adjudication, and these results do not isolate human input perfectly.

Resample each continuous piece uniformly at 100 samples/s. Initialize wheel angle/yaw once at window start, assume initial lateral velocity/roll/tyre loads zero, then discard 5 s warmup and 1 s trailing filter transient; no logged motion is fed back after initialization. Score a third-order zero-phase Butterworth bandpass, 0.3–3 oscillations/s, identically for road and simulation. Windows ending at segment boundaries sacrifice data to protect against hidden gaps. Spectral estimates use 10 s Hann records with 9 s overlap, exact 0.5/1.0/1.5/2.0 oscillations/s bins. They have few effective independent averages: low coherence or small command excitation is an **evidence limitation**, never a passing bin.

## Gate 1: FAIL

Requirement: command-to-wheel and command-to-yaw-acceleration gain within 0.8–1.2 of road, phase error within ±20°, and bandpassed correlation at least 0.8, for every controller and both speed bands. **0 of 12 controller × speed × output cells pass**, even using the optimistic median-window summary. A strict per-block requirement cannot pass when this summary fails. Detailed per-window bins, input power and coherence are in [gate1.json](../shake/gate1.json). Each table entry below is gain ratio / phase error in degrees, at the stated oscillations per second.

| Controller | mph | Output | Windows | Correlation | 0.5/s | 1.0/s | 1.5/s | 2.0/s |
|---|---|---|---:|---:|---|---|---|---|
'''
  for r in g1['summary']:
    values = ' | '.join(f"{b['median_gain']:.2f} / {b['median_phase']:+.1f}°" for b in r['bins'])
    text += f"| {model[r['model']]} | {'>45' if r['speed'] else '25–45'} | {'wheel' if r['output']=='wheel' else 'yaw lateral'} | {r['windows']} | {r['median_corr']:.3f} | {values} |\n"
  text += '''
![Held-out gain and phase](../shake/gate1.png)

The model misses stock most severely; high correlation on S5 does not rescue incorrect gain and phase. Above 45 mph, S5 yaw gain is 0.75 and phase error −43.2° at 2 oscillations/s. At 0.5 oscillations/s S5 wheel phase is +83.5°. These exceed the ±20% and ±20° gates. Some stock spectral bins have weak command/output coherence; the cure is independent excitation, not claiming those bins pass.

## Gate 2: FAIL

Runs the actual V0 class, NNLC with stock/S3/S5 JSON, each model's actual response shaper, and OdysseySteer through the existing harness/controller imports; no controller law was approximated. Stock here means **stock NNLC**, not NNLC disabled. Uses logged desired curvature and model future-plan arrays with logged torque/vehicle/delay calibration and pitch. Plant wheel angle/rate feed the real controller; quantization is 0.1°. There is no replayed wheel/body feedback in the closed loop.

Each 20 s trial resets controller state and warms up for 5 s. Initial integrator/history and true driver interaction are not reconstructed. Model/lane arms follow monotonic-clock block modulo 3/2. Desired curvature already carries the historical lane correction; do not add the new 0.5 s lane smoothing. OdysseySteer is instantiated by the inherited config with optional angle/kick compensation disabled in these >25 mph trials. Its currently edited runtime implementation was exercised, not a frozen historical checkout. All Params writes are redirected into the permitted scratch directory. These are material replay limitations; no exact reconstruction of the 9 November runtime is claimed.

Requirement: each RMS, crossings/s, and median absolute command 0.1 s after crossing within 0.75–1.25 of road, S5 peak within 0.3 oscillations/s, and the requested lane-off ordering. **0 of 3 controllers passes the metric bundle.** Ratios below are median paired-window simulation/road ratios, not ratios of pooled statistics.

| Controller | Windows | Crossings/s ratio | Size at +0.1 s ratio | Command RMS ratio | Wheel RMS ratio | Body RMS ratio | Median absolute peak error (oscillations/s) |
|---|---:|---:|---:|---:|---:|---:|---:|
'''
  for r in g2['summary']:
    values = ' | '.join(f"{r['ratios'][k]:.3f}" for k in ('crossings_s', 'after_crossing', 'command_rms', 'wheel_rms_deg', 'body_rms'))
    text += f"| {model[r['model']]} | {r['windows']} | {values} | {r['median_abs_peak_error']:.3f} |\n"
  text += '\nRoad/simulation median absolute statistics, respectively (body RMS includes equivalent push on 80 kg):\n\n'
  for arm in range(3):
    group = [r for r in g2['rows'] if r['model']==arm]
    vals = {side: {k: float(np.median([r[side][k] for r in group])) for k in group[0]['road']} for side in ('road', 'sim')}
    a,b=vals['road'],vals['sim']
    text += (f"- {model[arm]}: crossings {a['crossings_s']:.2f}/{b['crossings_s']:.2f} per second; command at +0.1 s "
             + f"{a['after_crossing']:.3f}/{b['after_crossing']:.3f}; body {a['body_rms']:.4f} m/s² ({80*a['body_rms']:.2f} N)/"
             + f"{b['body_rms']:.4f} m/s² ({80*b['body_rms']:.2f} N) RMS.\n")
  text += '\nLane-off descriptive ranking (different roads, not causal matched comparisons):\n\n'
  for arm in range(3):
    group=[r for r in g2['rows'] if r['model']==arm and r['lane']==0]
    if group:
      a=float(np.median([r['road']['body_rms'] for r in group]))
      b=float(np.median([r['sim']['body_rms'] for r in group]))
      text+=f'- {model[arm]}: n={len(group)}, road {a:.4f} m/s² ({80*a:.2f} N), simulation {b:.4f} m/s² ({80*b:.2f} N) RMS.\n'
  text += '''
The S5 peak subcheck passes in all **6 of 6 windows**, with maximum error **0.195 oscillations/s** and median error **0.000 oscillations/s** (roughly 0.098 oscillations/s spectral resolution), but peak coincidence under the logged plan is not proof that the feedback limit cycle is right; check the per-window errors in [gate2.json](../shake/gate2.json). No controller-design decision is authorized by this partial subcheck. The lane-off descriptive ordering does not override failed amplitude/crossing gates or route/speed confounding.

![Closed-loop ratios](../shake/gate2.png)

## What the earlier simulator missed—and what the ablations actually prove

The old calibrated/video plant is a friction-thresholded first-order scalar steering response followed by another first-order body response. It has no explicit steering inertia, presliding state, tyre-force relaxation, lateral velocity, yaw inertia or roll. It can fit slow tracking while suppressing fast alternating command energy. This is a structural reason its slow-fit calibration was insufficient, but **a unique element causing the full tenfold forecast miss has not been established**. The old locked forecast was a controller-difference jerk forecast (+0.04 versus +0.34 m/s³), not the same statistic as the open-loop RMS comparison here. Do not divide the following values and call it a replication of that forecast miss.

Training shake window: route 1ac, block 2, monotonic start 453.453 s; selected by maximum observed yaw-band RMS among eligible 20 s windows in that block. This is diagnostic selection on training data, not validation. Its RMS comparison is:

| Variant | Yaw lateral RMS m/s² | Equivalent RMS push on 80 kg |
|---|---:|---:|
'''
  for name, row in diagnostic['ablation'].items():
    text += f"| {name} | {row['yaw_rms']:.5f} | {80*row['yaw_rms']:.2f} N |\n"
  text += '''
![Training shake overlay](../shake/training_shake.png)

Replacing the physical model by the old scalar plant reduces this window's yaw signal from 0.13787 m/s² (11.03 N) to 0.05240 m/s² (4.19 N), versus road 0.15990 m/s² (12.79 N). It demonstrates fast-response attenuation, not a tenfold jerk explanation. Removing friction increases motion rather than making the shake disappear. Removing roll changes **no yaw motion**, because roll is one-way in this model; its assumed seat-specific-force RMS changes from 0.08970 m/s² (7.18 N) to 0.08261 m/s² (6.61 N). Shortening motor lag to the measured 0.024 s increases yaw RMS to 0.35480 m/s² (28.38 N). Shortening tyre relaxation reduces it to 0.08423 m/s² (6.74 N). These large, conflicting sensitivities expose non-identifiability. **The required “remove one element, shake disappears” causal gate is not met.**

Observed NNLC near-zero command expansion, S5's doubled error gain, lane-heading noise and the missing fast plant dynamics are all plausible contributors. The present experiment cannot rank their causal contribution. A closed-loop factorial ablation of the shaper, model, historical lane correction and independently identified plant is still needed. Claiming a roll resonance caused the whole failure would overstate this evidence.

## Quiet deadband plus minimal bump: deferred

No candidate module, default-off runtime parameter, tuned settings, lane-weave trade curve, or locked forecast is delivered: the brief explicitly conditions these on both gates passing. `ladder_diagnostic.json` is an invalid-plant inversion diagnostic, **not** a locked controller forecast.

The design to test once the plant validates is: low-pass the actual path/request error; enter a positive/negative correction state only after a sustained high threshold, release below a smaller threshold; emit one signed, bounded breakaway pulse, then wait for measured wheel response and a refractory interval before another pulse. Stay quiet inside the release band, and reset on invalid input/override. Trigger on sustained path need, not every command zero crossing. Existing `steer_deadband.py` kick adds signed assistance around every request, while its bump concentrates assistance near zero; `odyssey_wander.py` friction assistance targets slow correction. Neither is evidence for a particular one-pulse setting. Read-only comparison only; no runtime edits.

## Perception ladder: specification, not executable on-device code

### What the literature supports

[Gianna et al., 1996](https://pubmed.ncbi.nlm.nih.gov/8749157/) measured a 67%-correct direction threshold for normal subjects of **0.0484 m/s² (3.87 N on 80 kg)** for acceleration steps with peak acceleration gradient 0.22 m/s³ (17.6 N/s). The same paper reports much larger thresholds for slow ramps: 0.152 and 0.264 m/s² (12.16 and 21.12 N) at gradients 0.028 and 0.079 m/s³ (2.24 and 6.32 N/s). These are stimulus-dependent thresholds, not a universal jerk cutoff and not sinusoidal frequency curves.

[Benson, Spencer and Stott, 1986](https://pubmed.ncbi.nlm.nih.gov/3790028/) tested 24 subjects with discrete 3 s cosine-bell velocity trajectories and found duration dependence consistent with sensitivity to acceleration plus jerk. [Valko et al., 2012](https://pubmed.ncbi.nlm.nih.gov/23015443/) measured frequency-dependent interaural translation/roll thresholds in 14 healthy subjects and three bilateral-ablation patients in darkness. Those tasks involve forced-choice direction recognition under controlled sensory conditions, not a driver answering whether a repeated vibration was felt. Their abstract does not supply a defensible numerical 0.3–3 oscillations/s translation threshold table, so none is invented here. The full journal page was access-blocked during research; recovering its figure/table values is an explicit remaining literature task.

[ISO 2631-1](https://www.iso.org/standard/7612.html) concerns whole-body vibration evaluation. Lateral seated vibration uses the Wd frequency weighting, also described in [de Winkel et al.'s full paper](https://repository.tudelft.nl/file/File_e43765bf-dcb9-4358-9f33-0e76ad3a707c); report frequency-weighted RMS and duration separately from unweighted peak force. A commonly used approximate comfort boundary is weighted RMS 0.315 m/s² (25.2 N RMS on 80 kg), **not** a detection threshold or an acceptable peak target for this driver. The full normative weighting coefficients were not retrieved here; the existing arbitrary bandpass is not labeled ISO compliant. Apply the licensed/published Wd transfer function before reporting an ISO result.

[de Winkel et al., 2023](https://pubmed.ncbi.nlm.nih.gov/36058166/) tested 23 participants with peak accelerations 0.4–2 m/s² (32–160 N) and jerk 0.5–15 m/s³ (40–1200 N/s). Their discomfort results depend on pulse shape, duration and direction; higher jerk did not simply mean more discomfort. These suprathreshold comfort data do not establish William's detection limit. Consequently, no literature-backed scalar “safe/unfelt jerk” number across 0.3–3 oscillations/s is claimed.

For a sine, peak jerk is `2*pi*f*peak_acceleration`; a ±4 N (±0.05 m/s²) signal at 0.3/1.3/3 oscillations/s has peak jerk 0.094/0.408/0.942 m/s³ (7.54/32.67/75.40 N/s). Thus identical peak push can feel different with frequency; measure each separately. William's road evidence already brackets an unnoticed/noticed difference around fast RMS 0.045–0.09 m/s² (3.6–7.2 N RMS), not a population detection threshold.

### Protocol

1. Use a dry, straight, level **closed test course**, with space to stop and no nearby traffic. Driver remains responsible and can cancel; a passenger administers responses. Driver explicitly arms each short block. Start at a steady 40 mph for calibration; 60 mph only after measured force and path envelopes are confirmed at that speed. Keep tyres/pressure/load/seat/head posture fixed and documented. The table below is an offline 60 mph example only.
2. Frequencies: 0.3, 0.5, 1.0, 1.3, 2.0, 3.0 oscillations/s. Candidate peak push ladder on 80 kg: **±0.25, 0.5, 1, 2, 4, 7, 9, 14, 20 N**, respectively **±0.003125, 0.00625, 0.0125, 0.025, 0.05, 0.0875, 0.1125, 0.175, 0.25 m/s²**. The lowest level is a proposed subthreshold test, not guaranteed imperceptible on a noisy road. The top is S5's typical push; it is optional and unnecessary once reliably detected. Do not reproduce its worst peaks.
3. Before blind trials, calibrate the lowest levels under measured-force feedback, one level at a time. Stop escalation on discomfort, excessive path displacement, angle/command saturation or poor achieved waveform; never infer command from the rejected model and inject it on the highway. Set prospective stop envelopes: measured peak 0.30 m/s² (24 N), lane-relative displacement 0.20 m, or driver cancellation. Use the lower of the envelope and the individually tolerated level; these are proposed experiment limits needing course/operator review, not an assertion of safety from this model.
4. Each stimulus: raised-cosine amplitude ramp, 2 s in/out, plus plateau `max(10 s, 6/f)` (20 s at 0.3/s; 10 s at 1/s and above), then at least 10 s quiet. Randomize phase/sign. Use an independent constant baseline controller and measured band-limited body motion; do not freeze lane control blindly for a 20 s run. Record both controller and injected commands so cancellation by lane keeping is visible. Subtracting sham noise statistically is preferable to subtracting a single noisy trace.
5. Discovery block: randomize frequency order; two ascending familiarization levels below discomfort, then randomize amplitudes within a bounded neighbourhood of the estimated threshold. Confirm with six repeats per tested level/frequency, 20% randomized zero-injection shams, spread over short sessions with breaks. Full nine-level factorial with six repeats and shams takes roughly 25–40 minutes per frequency before extra breaks; use adaptive bracketing to avoid hours of unnecessary discomfort. No force number or condition is displayed until after the yes/no response, to avoid suggestion; afterward show achieved peak/RMS push and waveform quality.
6. Fit `P(yes|a) = false_positive + (1-false_positive-lapse)*logistic((log(a)-mu)/s)` using **achieved** force amplitude. Estimate false positives from shams and lapse from clear comfortable positives. Define threshold as the midpoint between floor and ceiling (approximately 50% detection after false-positive correction), report 75% detection too, and bootstrap whole trials/session blocks for a 95% interval. If only yes/no without enough shams/repeats, report a bracket, not a precise threshold. A two-interval forced-choice extension would reduce response bias but doubles exposure/time. Keep detection, comfort and nausea as separate responses.

The existing maneuver suite needs randomized blind schedules/shams, per-block arming/cancel, measured-force and path envelopes, timestamped achieved waveform/arm/response logging, calibration status, session persistence and force display after response. Another worker owns that code; none is implemented here. The existing wheel-press abort can false-trigger on self-motion, so cancellation logic needs explicit driver input plus a separately reviewed torque criterion rather than disabling all aborts.

### Command conversion: rejected-model diagnostic only

The invalid model was inverted by sinusoidal sweep at 60 mph, measuring the steady first harmonic of `v*yaw`. It is nonlinear and can have multiple/no roots; the inversion chooses the first grid-bracketed root. It includes Honda slew, deadband and stick-slip. It predicts the following normalized command amplitudes; **these are unsuitable for on-road use** and demonstrate why calibration cannot be skipped. A dash means the target was not bracketed below full-scale command, not a proven physical impossibility.

| Oscillations/s | ±1 N (±0.0125 m/s²) | ±7 N (±0.0875 m/s²) | ±20 N (±0.25 m/s²) |
|---:|---:|---:|---:|
'''
  for hz in (.3, .5, 1., 1.3, 2., 3.):
    values=[]
    for force in (1.,7.,20.):
      value=next(r['unvalidated_command_amplitude'] for r in ladder['rows'] if r['times_per_second']==hz and r['peak_newtons_on_80kg']==force)
      values.append('—' if value is None else f'{value:.3f}')
    text+=f"| {hz} | {' | '.join(values)} |\n"
  text+='''
The experiment should identify a measured local amplitude-to-force response at each speed/frequency before blind testing, then adapt bounded commands to achieved force. No validated mapping to seat/head force exists yet. All nine levels are retained in [ladder_diagnostic.json](../shake/ladder_diagnostic.json), with `authorized_for_road: false`.

## The next excitation and what I need from William

**Recommendation: one independently armed, low-amplitude identification session on a closed straight course, then a separately held-out repeat.** First fix sensor-frame/time alignment and motor sign/torque calibration. Do not ask another road exposure to compensate for a bad measurement pipeline.

- Confirm model year/trim, fitted tyre make/model/size and cold pressures; alignment or suspension changes. For each 9 November route, give occupant masses and seating rows/sides, cargo masses/positions, fuel estimate, whether second/third-row seats were present/folded, and notable road/wind conditions. Measure device pose/height and driver's seat/head heights. These determine loaded mass, axle distribution and roll lever arms.
- Calibration: collect synchronized raw gyro/acceleration with sensor source IDs, extrinsics and sensor timestamps, actual CAN command, motor magnitude, wheel angle/rate and driver torque. Add a seat-rail/body-frame IMU and preferably a second rigidly mounted sensor at a known vertical separation. Their acceleration difference plus roll rate separates rotational lever-arm motion from translation/gravity. An external column/rack torque calibration is needed to recover absolute physical friction/inertia; otherwise report normalized/gauge-equivalent values.
- First identify EPS map and breakaway with isolated alternating normalized command pulses **0, 0.02, 0.03, 0.045, 0.06, 0.08**, 0.25 and 0.5 s each, 3 s quiet between, three repeats each sign. Start at 40 mph; repeat at 60 mph only after measured envelopes hold. Abort instead of escalating automatically if baseline steering cancels the pulse or response is unclear. The motor magnitude identifies onset/cap/lag; wheel angle/rate identifies breakaway and presliding hysteresis. Cap characterization above this range requires a separate gentle sustained curve/controlled steering test; it is not identified by small straight-road pulses.
- Next use a zero-mean multisine/chirp over **0.3–3 oscillations/s**, 60 s each at nominal amplitudes **0.03, 0.045, 0.06**, 15 s quiet between, twice at each speed if achieved forces/path remain inside the prospective envelopes. Only increase to 0.08 with explicit re-arming after reviewing the prior response. These amplitudes may not overcome stiction; that is a result. Use randomized phases and retain injected input as an instrumental variable against closed-loop bias. Separate speed/load sessions identify aligning response, tyre/yaw phase and speed dependence; roll needs independent calibrated IMUs. If safe excitation cannot produce adequate coherence near 2–3/s, the gate remains unidentifiable there rather than raising torque until it passes.
- Reserve a distinct repeat/session as untouched validation before fitting. The current 1ad/1ae failure patterns must not guide a model that later calls those same routes unseen. Predeclare window exclusions using true observed driver actions, not all steeringPressed flags.

## Reproduction, checks and exact continuation point

`extract.py` creates augmented NPZs only under the allowed scratch directory. `analyze.py --fit` trains then freezes `fit.json`; without `--fit` it only scores that frozen model. `closed.py` runs actual controllers; `eps.py` and `diagnostics.py` identify training-only discrepancies; `ladder.py` produces rejected-model inversion examples; `write_report.py` regenerates this document and gate-2 plot. Never interpret the report generator as validation.

Set `PYTHONPATH=$PWD:<session scratch>/shake/site`, `MPLCONFIGDIR=<session scratch>/shake/mpl`, then run `.venv/bin/python -m tools.odyssey_sim.shake.<module>`. Scratch dependencies: NumPy, SciPy, Numba, Matplotlib. The code has a plain NumPy fallback if Numba is unavailable; full fitting is much slower. Primary regression checks use `.venv/bin/python tools/test_runner.py tools/odyssey_sim/shake/test_physics.py`; **4 tests pass**: unforced equilibrium, sub-breakaway stillness, sign symmetry/finite motion, and identical full-series versus incremental closed-loop plant stepping. Ruff checks the entire shake directory. The manifest records end-of-run file hashes; it is not an archived proof of the concurrently edited runtime source at process start. Model fitting and 17 held-out open/closed-loop trials actually ran; this is not a stubbed test.

Self-review: an initial unconstrained aggregate-lag fit was checked against measured motor response and explicitly rejected as a physical EPS estimate; the orientation fit was checked for positive damping and rejected; requested spectral frequencies were made exact; closed-loop state continuation was tested against batch integration. No further tuning was done against the exposed holdouts. The implementation is a research prototype with initial-state, human-input, map and sensor-identification limitations above.

**Exact next step:** carry sensor source IDs and timestamp/extrinsic calibration through extraction, verify vehicle-frame gyro/acceleration against the known yaw sign and stationary gravity, then identify motor/reversal response independently and predeclare a fresh validation session. Until then both gates remain failed, the tenfold causal explanation remains unproven, and no bump tuning or road forecast is justified.
'''
  report=OUT.parent/'reports/report_shake.md'
  report.write_text(text)
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  fig, ax=plt.subplots(figsize=(10,4))
  keys=list(g2['summary'][0]['ratios'])
  for i,r in enumerate(g2['summary']):
    ax.bar(np.arange(len(keys))+(i-1)*.24, [r['ratios'][k] for k in keys], width=.23, label=model[i])
  ax.axhspan(.75,1.25,color='green',alpha=.13)
  ax.set_xticks(np.arange(len(keys)), ['Crossings/s','Size at +0.1 s','Command RMS','Wheel RMS','Body RMS'])
  ax.set_ylabel('Simulation / road (paired-window median)')
  ax.legend()
  fig.tight_layout()
  fig.savefig(OUT/'gate2.png',dpi=150)
  paths=list(OUT.glob('*.py'))+list(OUT.glob('*.json'))
  runtime=['openpilot/sunnypilot/selfdrive/controls/lib/latcontrol_torque_v0.py',
           'openpilot/sunnypilot/selfdrive/controls/lib/nnlc/nnlc.py',
           'openpilot/sunnypilot/selfdrive/controls/lib/odyssey_steer.py',
           'tools/odyssey_sim/replay/harness.py','tools/odyssey_sim/closedloop/controller.py']
  paths += [Path(p) for p in runtime]
  (OUT/'manifest.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if p.name!='manifest.json'},indent=2))
  print(report)


if __name__ == '__main__':
  main()
