# Odyssey output roughness, version 2

Use this offline screening metric instead of rejecting a controller for command
reversals. It measures physical outputs after the EPS. It is a provisional index,
not a validated probability of what William notices. The logged torque sensor has
no established conversion to N·m, and hands-on status is not observable reliably.
Missing hand torque must stay unknown.

```python
from tools.odyssey_sim.perception.metric import (
  Thresholds, perceptible_roughness, from_simulator, sluggishness,
)

result = perceptible_roughness(
  dt=0.01,
  angle_deg=fine_angle,             # tools.odyssey_eps_report.fine_angle
  angle_rate_deg_s=can_angle_rate,  # preferred over differentiating angle ticks
  lat_accel=v_ego * yaw_rate,       # carControl.angularVelocity[2]
  command=can_steer_torque / 4096,   # optional diagnostic only
  valid=eligible,                  # active, no override/blinker, supported speed
)
print(result['roughness'], result['p95_window'], result['scored_windows'])

# Materialize plant.step outputs; no sensor model is hidden here.
sim_result = from_simulator(states, dt=0.01, command=commands)

# Independent, event-conditioned lag diagnostic, same signs and units:
exit_result = sluggishness(dt=0.01, actual=actual_lat_accel,
                          reference=desired_lat_accel, threshold=0.1)
```

## Stable interface and input contract

`perceptible_roughness(*, dt, angle_deg, angle_rate_deg_s=None, lat_accel=None,
hand_torque_nm=None, command=None, valid=None, thresholds=None, window_s=10.,
hop_s=5.)` returns a dictionary. `from_simulator(states, *, dt, command=None,
**kwargs)` forwards `angle_deg`, `angle_rate_deg_s`, `lat_accel`. `Thresholds` is a
frozen dataclass; use `dataclasses.replace` for sensitivity checks.

Arrays must be finite, one-dimensional, same length and on a uniform time grid.
Split at log gaps, segment discontinuities, mode switches and driver overrides;
never concatenate separated eligible samples. Alternatively mark discontinuities
false in `valid`, which excludes any crossing window. Interpolate onto the common
grid first; use zero-order hold for command and linear interpolation for continuous
outputs. Indexing CAN samples at 100 Hz is only an approximation to this contract.
Do not score warm-up transients as controller roughness. Empty or too-short eligible
data returns `roughness=None`, not zero. All public functions are offline analysis,
not a 100 Hz runtime hook.

Outputs include `roughness` (root mean window excess energy), `p95_window`,
`scored_windows`, `coverage_channels`, `bands_hz`, `thresholds`, and `windows`.
Each window contains per-band physical `rms`, dimensionless `threshold_ratio`,
`excess_energy`, `channel_energy`, and optional `command_reversals_hz` (zero samples
are skipped). Compare runs only with identical channels, thresholds, masks, road,
speed and sampling. A wheel-only score is not comparable to a wheel/body/hand score.
Unsupported bands above Nyquist are omitted and reported. Missing hand coverage
means haptic roughness is not fully assessed. Finite arrays required even where
valid is false: split around missing measurements first.

## Definition

Ten-second Hann windows, 5 s hop, with mean and linear trend removed. One-sided
PSD energy is integrated into 0.1–0.5, 0.5–1, 1–2, 2–5, 5–10 Hz. The first band is
reported but **excluded from roughness**: ordinary curves, slow unwind and wander
need a reference to distinguish intended motion from error. Keep those tracking
and sluggishness costs separately. The score therefore does not excuse wander.

For channel c, band b: `q² = integral(PSD_c(f) / threshold_c(f)² df)`;
`E_cb = max(q² - 1, 0)`. Sum scored bands within channel, take the maximum across
channels to avoid counting wheel/body manifestations twice. Aggregate windows as
`sqrt(mean(E))`; the p95 is `sqrt(percentile(E, 95))`. This is band RMS excess,
not clipping individual Fourier-bin amplitudes or a probability. Frequency
resolution is 0.1 Hz; short impacts under 10 s are diluted. Keep event traces and
p95 as well as the aggregate. Overlapping windows are not independent trials.

Default output screening thresholds:

| Channel | Threshold | Basis and limit |
|---|---|---|
| Wheel, <3 Hz | max(0.1°, 14% of absolute window mean angle) RMS | Relative scale informed by steering-angle psychophysics; 0.1° floor is an engineering hypothesis, not published absolute JND. |
| Wheel, >=3 Hz | smaller of above and angle equivalent of 0.2 m/s² RMS tangential rim acceleration at radius 0.19 m | Empirical background allowance, informed by manual-route rate spectra; not a human detection threshold or a measured wheel radius. |
| Body | max(0.05 m/s², 0.3 m/s³ / (2πf)) RMS | Requires appreciable acceleration AND jerk; engineering screening values, not ISO limits or universal perception constants. |
| Hand | max(0.1 N·m, 15% of absolute mean torque) RMS | Relative JND informed by literature; absolute floor is provisional; requires calibrated N·m input. |

When rate is supplied, wheel displacement PSD comes from `PSD_rate/(2πf)²`, which
avoids differentiating the quantized angle or the fine-angle clamp. Rim acceleration
is `radius * radians(angle_accel)`. The plant's **steering_accel is m/s²**, an
angle-equivalent lateral acceleration; it is not angular acceleration and is never
used as such here. The 0.2 m/s² high-band floor exceeds manual-route 5–10 Hz p99
background (0.145/0.183 m/s²); 2–5 Hz manual p99 reaches 0.246. This allowance
includes road/driver/sensor motion and cannot isolate quantization noise.

## Lag, validation and use by harnesses

`sluggishness(*, dt, actual, reference, threshold, max_lag_s=4.)` estimates the
nonnegative lag minimizing aligned MSE and returns excess-error RMS and integral.
Select events first, apply identical low-pass filtering to both signals, and use
identical physical units/signs. Constant demand yields unknown lag. Gain/bias
mismatch can bias lag. A model-demand reference omits model/planner delay; use
road demand to assess the whole exit behavior. This metric is diagnostic and is
not added to the main roughness index.

Replay must run candidate CAN commands through a plant before scoring candidate
physical outputs. Logged outputs belong to the controller actually driving;
attaching different replay commands to those same outputs cannot compare smoothness.
Closed-loop and calibration harnesses can call `from_simulator`; use matched
windows and both supported plants. Restrict numerical claims to the plant's
validated speeds (this analysis uses >=8 m/s, not parking). Sensor-noisy logs and
noise-free plant outputs have different backgrounds: compare candidate deltas
within each domain rather than absolute scores across domains.

Run threshold sensitivity at 0.5× and 2× physical floors. Report tracking,
saturation, lane wander, and road-relative entry/exit response beside roughness.
Do not tune away tracking to get a zero roughness score. The scoring bands do not
cover all human tactile perception (published wheel studies extend above 300 Hz).
No causal hand-transmission percentage or human perceptibility certification is
possible from these logs alone.

The research, failed attempts and numerical evidence are in
`scratchpad/report_perception.md`, `perception_results.json`,
`perception_attempt1.json`, `perception_counterfactual.json`,
`perception_sensitivity.json`, and `perception_events.json` under the shared scratch
root given in the task brief. Eight behavior checks run with
`PYTHONPATH=$PWD .venv/bin/python tools/test_runner.py tools/odyssey_sim/perception/test_metric.py`.
