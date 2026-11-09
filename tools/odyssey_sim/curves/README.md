# Curve transition research and integration map

`transition.py` is the only runtime dependency (NumPy). It defaults off. Everything
else here is offline analysis. Neither the model/observer nor the controller loop
passes the calibration gate; these are experimental add-ons, not a deployment.

## Exact integration contract for the lead

1. Copy `transition.py` into `openpilot/sunnypilot/selfdrive/controls/lib/curve_transitions.py`.
   Register **SteerCurvePreview** and **SteerCurveReturn** as off-by-default bool
   params in the canonical param-key registry. Return is a separate experiment.
   Re-read while offroad; reset both instances on toggle, disengagement, model
   invalidity, blinker, calibration invalidity, or a control-clock discontinuity.
2. In `controlsd`, construct two instances:
   `pre = CurveTransitions(enabled=preview_param, preview=True)` and
   `ret = CurveTransitions(enabled=return_param, return_mode='active')`.
   Do not enable return for the first road experiment. A release experiment uses
   `return_mode='release'`; this removes at most 0.20 normalized holding torque,
   so it is a **partial release** on larger stock commands.
3. For every 100 Hz frame, build the arguments below from the current CS, VM,
   vehicleParameters, and most recent valid modelV2. No synthetic future data:

   ```python
   v = CS.vEgo
   target = new_desired_curvature * v * v
   planned_angle = math.degrees(VM.get_steer_from_curvature(
     -new_desired_curvature, v, lp.roll))
   common = dict(
     speed=v, angle_deg=CS.steeringAngleDeg-lp.angleOffsetDeg,
     rate_deg_s=CS.steeringRateDeg, planned_angle_deg=planned_angle,
     plan_t=ModelConstants.T_IDXS, plan_accel=sm['modelV2'].acceleration.y,
     active=CC.latActive and model_valid and not blinkers,
     driver_pressed=CS.steeringPressed, driver_torque=CS.steeringTorque,
     plan_age=now_mono-sm.logMonoTime['modelV2']*1e-9, dt=DT_CTRL)
   target, _ = pre.update(command=0., target_accel=target, **common)
   new_desired_curvature = target / max(v*v, 0.1)
   ```

   Insert this **before** `clip_curvature` in `controlsd.py` (currently line 149),
   For shadow logging, run a separate preview instance with enabled=True and
   log its target while passing the original curvature to control. The off
   param must gate consumption of the adjusted target, not permit intervention
   merely because shadow computation is running.
   Then use the same clipped curvature for LaC, `OdysseySteer`, telemetry, and
   actuator curvature. Keep all existing curvature/jerk limits. `plan_age` must
   use the same monotonic clock as the model message. Account for camera/model
   processing latency separately if that timestamp convention changes; do not
   add the learned lateralDelay to plan_age. Do not change lateralDelay: it also
   changes NNLC's own future inputs, which is a different experiment.
4. Return, if separately tested, runs **after** existing OdysseySteer angle and
   deadband processing but **before** characterization torqueOffset is added.
   Recompute planned_angle from the **clipped** desired curvature and update
   common accordingly. In OdysseySteer.apply, normalize once:

   ```python
   _, u_accel = ret.update(command=-steer / steer_max,
                          target_accel=desired_curvature*v*v, **common)
   steer = -u_accel * steer_max
   ```

   The runtime class uses OP acceleration-positive torque; Honda `steer` and
   torqueState.output have the opposite sign. Angles retain Honda sign.
   Preserve Honda CAN/panda limits. Pass intervention/rate limitation back to
   LaC's `steer_limited`/PID freeze path on the next frame; otherwise the stock
   integrator can wind up against return assistance. Freeze during assistance,
   including its ramp-out, and reset appropriately on mode changes. Disable
   curve add-ons in Lateral Maneuver Mode. Do not stack return with low-speed
   angle control: this experiment is for >=5 m/s only.
5. Record the original and adjusted curvature, preview correction, return delta,
   mode, applied torque, model age, wheel angle/rate, driver torque and v*yaw.
   The module's 0.4 s rate guard avoids issuing an assist pulse through straight;
   it is not a proof against physical overshoot. Never bypass final limits to
   preserve a requested return pulse.

The offline factory adapter demonstrates units, conversion and call ordering;
its 10 Hz immutable caches cannot reproduce native model age or CAN timing.
Use separate preview-only/return-only instances once each frame. Passing both
options to one instance is useful for standalone experiments, but does not solve
LaC call ordering in production.

## Reproduction

From repository root with `PYTHONPATH=$PWD` and `.venv/bin/python`:

- `-m tools.odyssey_sim.curves.characterize`: event timing, releases, future forecast.
- `-m tools.odyssey_sim.curves.experiment`: 58 matched curve windows, 6 variants,
  plant-v2 closed loop and fixed-motion real-LaC replay.
- `-m tools.odyssey_sim.curves.native_replay`: return postprocessing on the validated
  native replay cache; preview **target** lead is separate from native LaC commands.
- `-m tools.odyssey_sim.curves.stress`: 30 sign/speed/mode return grid cases.
- `-m tools.odyssey_sim.calib.evaluate --logged-plan-only --controller tools.odyssey_sim.curves.controller:preview --output curves_preview.json`.
- `-m tools.odyssey_sim.calib3.evaluate --controller tools.odyssey_sim.curves.controller:preview --output curves_preview.json`.
- Substitute `release`, `active`, `combined`, `preview_full`, or `inverse` in the
  calib hook with a distinct output filename to reproduce rejected generations.
- `.venv/bin/python tools/test_runner.py tools/odyssey_sim/curves/test_transition.py`.
- `.venv/bin/ruff check tools/odyssey_sim/curves`.

Results and generation decisions live in scratchpad/report_curves.md and
scratchpad/curves/. Existing cache artifacts are never overwritten by extraction.
