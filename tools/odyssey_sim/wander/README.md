# Highway wander experiments

Experimental, default-off modules for the lead to integrate. No van changes have
been made. `runtime.py` needs only Python stdlib and NumPy. It contains a bounded
camera/body-yaw observer and speed-scheduled desired-acceleration correction,
and three error-conditioned friction-assistance experiments. These are research
candidates; centimetre-level road tracking has not been demonstrated.

## Map and reproduction

- `runtime.py`: scalar runtime API, documented units/signs and reset behavior.
- `adapter.py`: real LatControlTorqueV0/NNLC adapters for calib's evaluator.
- `research.py`: legacy-cache diagnosis, command replay, lane stress simulations.
- `native_replay.py`: 20 Hz origin geometry + body-yaw command replay.
- `logged_plan.py`: real controller + fitted plant + output perception screen.
- `test_runtime.py`: seam tests for bounded corrections, disable/takeover,
  missing frames, curved-road equilibrium, jitter reduction, pulse duty, speed.

From repository root, with scratch caches produced by calib/calib3:

```sh
PYTHONPATH=$PWD .venv/bin/python tools/test_runner.py tools/odyssey_sim/wander/test_runtime.py
.venv/bin/ruff check tools/odyssey_sim/wander
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.wander.research
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.wander.native_replay
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.wander.logged_plan
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.calib.evaluate --logged-plan-only --controller tools.odyssey_sim.wander.adapter:inverse --output wander_inverse.json
```

`research.py` uses a simplified PID for lane stress rejection only; logged-plan
adapters use the actual running V0/NNLC classes. Neither has passed recorded
baseline closure. No deployment ranking follows from these scores.

## Integration contract for the lead

1. Add bool params `OdysseyWanderLane` and `OdysseyWanderFriction`, both false by
   default; param reads stay outside these modules. Own one instance of each
   class in OdysseySteer/controlsd. Reset on disengagement, steeringPressed,
   blinkers/lane changes, maneuver mode, invalid sensors, or A/B arm transition.
2. Compute fresh camera geometry at **x=0** from the two inner lane lines in
   calibrated model coordinates. Use calib3's train-selected quadratic fit
   through 25 m (`geometry.origin(..., degree=2, horizon=25)`), not a lookahead
   offset. Averaged inner-lane y, slope and curvature share the calibrated body
   yaw sign. Do not apply extrinsics again. The selected offline -0.3 s geometry
   shift is NOT authorization for future data or a runtime delay trick.
3. At 100 Hz, call `LaneCentering.update` with speed m/s, yaw in rad/s from the
   validated carControl angularVelocity channel, camera offset metres, heading
   radians, road curvature 1/m, dt in seconds. Set `camera_new` once per model
   message; hold the latest measurements between messages. Never re-apply a
   20 Hz measurement five times. `valid` must require both inner-lane confidence
   >0.6, fresh model and yaw, plausible lane width, no steering takeover/blinker,
   no maneuver mode, and no lane-change state. Modules additionally reject
   speeds outside 18–40 m/s, offsets >0.6 m, road acceleration >1.2 m/s²,
   nonfinite data, missing camera >0.25 s, or dt outside 1–120 ms.
4. The returned acceleration is **additive**: `curvature += correction/vEgo**2`
   before `clip_curvature` and `LaC.update`. Feed the same corrected curvature
   to OdysseySteer. Do not modify only its torque feedforward, leaving the PID
   setpoint behind. Keep this arm shadow-only until observer issues in the
   report are resolved. Do not stack it with another lane integrator.
5. Friction assistance acts after V0/NNLC/OdysseySteer's other shaping, before
   Honda's final torque/rate/safety limiter. Its command is acceleration-positive;
   runtime torque is the opposite. Exact sign adapter:

   ```python
   shaped_torque = -friction.update(
     -stock_torque, target=corrected_curvature * vEgo**2,
     actual=body_lat_accel, speed=vEgo, rate=steeringRateDeg,
     dt=DT_CTRL, enabled=friction_param_and_ab_arm, valid=eligible,
   )
   ```

   `body_lat_accel = vEgo * validated_yaw`. Check it against torqueState's
   VehicleModel actual on the highway first: they have near-unit fitted gain
   but different roll/bias terms. The existing offline adapter uses the fitted
   steering state; therefore using raw body acceleration is an explicit
   remaining integration mismatch, not an already validated equivalent. A lead
   may start with the controller's actualLateralAccel for fidelity to evaluated
   behavior, while shadow-logging the body-based version. Never use VSA LAT_ACCEL.
6. Start with `FrictionAssist('inverse', scale=1)`. Addition is at most 0.22,
   slewed at 1.5 command/s; v2 produces about 0.06–0.12 at highway speeds.
   Target error is smoothed with tanh(error/0.06). Do not stack the old 0.03 kick
   in the same experiment. Pulse/dither are simulation-only candidates for now.
7. Log arm identity, enabled/valid reason, fresh-camera flag, raw and estimated
   offset/heading, observer bias, original/corrected curvature, body and VM
   acceleration, pre/post-assistance torque, final CAN command, fine angle/rate,
   takeover and saturation. Existing A/B clock alone does not prove the active
   controller ran the change; verify nonzero post-minus-pre command in logs.

Runtime gates fall back immediately to stock and clear memory. Parameter changes
must reset both instances. Host benchmark does not substitute for 3X timing.

`planner_fit.py` reproduces the descriptive regression used to challenge the
inherited planner gains. `native_replay.py` excludes nonzero logged maneuver
injection; the cache lacks full maneuver-mode state, so it cannot reproduce the
complete runtime eligibility gate. Treat replay fractions as shadow diagnostics.
