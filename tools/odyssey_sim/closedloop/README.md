# Odyssey closed-loop research harness

Run from the odyssey-pilot root, on the existing `odyssey-sim` branch. No production imports are copied or edited. All caches/results go in the supplied scratchpad; Params uses `simclosed`, `simclosedodyssey`, `simclosedtiny`, `simclosedvalidation`, or `simclosedcheck`, never the real/default namespace. Setup writes block and controller construction asserts settings.

```
PYTHONPATH=$PWD .venv/bin/python tools/odyssey_sim/closedloop/data.py
PYTHONPATH=$PWD .venv/bin/python tools/odyssey_sim/closedloop/check.py
PYTHONPATH=$PWD .venv/bin/python tools/odyssey_sim/closedloop/validate.py
PYTHONPATH=$PWD .venv/bin/python tools/odyssey_sim/closedloop/rollout.py --plant provisional
PYTHONPATH=$PWD .venv/bin/python tools/odyssey_sim/closedloop/rollout.py --plant odyssey
.venv/bin/ruff check tools/odyssey_sim/closedloop
```

Tinyphysics dependencies are in scratchpad/closedloop-venv, leaving the repo venv unchanged. Its launch (adds the repo's editable dependencies using its .pth files):

```
PYTHONPATH=$PWD /private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/closedloop-venv/bin/python -c 'import sys,site,runpy; site.addsitedir(".venv/lib/python3.12/site-packages"); sys.argv=["rollout","--plant","tiny"]; runpy.run_module("tools.odyssey_sim.closedloop.rollout",run_name="__main__")'
```

`data.py`: all six local routes decoded via FILE LogReader at approximately 10 Hz. CSV challenge columns plus actual, active/driver masks and live parameters; NPZ adds real modelV2 arrays. `targetLateralAcceleration` comes from `carControl.actuators.curvature * vEgo²`, the actual LaC input. `rawCurvature` retains controlsState.desiredCurvature before the upstream limiter. This isolates controller comparison with the upstream plan and limiter held fixed.

`controller.py`: real V0 LatControlTorque -> extension -> Odyssey NNLC; inverse VehicleModel reconstructs wheel feedback from simulated wheel-derived acceleration with the logged ratio, stiffness, roll and offset. Real NN future acceleration/orientation arrays are passed in unchanged. 100 Hz control with 10 Hz logged plan/state zero-order hold; no driver input or lateral planner rerun. Config matrix changes one factor at a time plus a combined angle/half-friction/kick arm. Baseline friction threshold is V0's 0.3; widened arm is 0.6. NN half-friction only scales the explicit torque-parameter friction, not the network's learned friction behavior.

`rollout.py`: imports comma's actual TinyPhysicsSimulator/Model for generic baseline. Native physics adapters use native 100 Hz control/plant steps and challenge cost formula on 10 Hz observations. Both discard ten seconds; tiny additionally uses comma's first 20 history frames and logged warmup actions through 10 seconds. A single fixed seed per route/window makes tiny comparisons share random draws, but does not establish statistical significance. Fitted params are frozen to closedloop_{odyssey,public}_plant_snapshot.json at sweep start.

Window selection: one 30-second contiguous window per route/band with >=3 seconds of originally active, unpressed samples in the scored final 20 seconds, chosen for highest mean absolute plan acceleration on those samples. Metrics only use that band/active/unpressed mask. Inactive context is simulated with control active to preserve continuous controller/plant evolution; these samples are excluded from costs. Thus this is a deliberately curve-heavy stress sample, not route-wide prevalence, causal takeover prediction, or representative average. Window time and scored sample counts are in JSON. Jerk and sign-change calculations require adjacent eligible samples. Physical torque metrics use native 100 Hz commands, tiny torque metrics use the applied 10 Hz commands.

Entry/exit: median nearest simulated |accel|=0.3 crossing within +/-2 seconds of each eligible planned crossing (positive means later); sparse unmatched events are omitted. This is a descriptive crossing statistic, not robust event matching; counts are mandatory. Saturation is |command|>=0.99, not production's speed/timer/safety-gated saturated flag. Flips require a sign crossing with >0.01 command change. No CAN torque-rate limits or EPS safety/driver limiter are modeled.

Fitted API adapter uses `steering_accel`, since production torqueState.actualLateralAccel is wheel-angle derived; inertial `lat_accel` is a separately lagged output. Its bicycle `angle_deg` is not directly fed into VehicleModel: both directions use the same real VehicleModel so feedback curvature units are exact. No plant proves low-speed performance; worker 2's model clamps below 5 m/s. The provisional plant has gain 1.47, deadband 0.03, friction 0.145, offset -0.055, roll gain 9.81 and lag 0.4 s, assumed rather than fitted.

Final feature arms use the real OdysseySteer.apply after V0/NNLC output, in actuators.torque sign. V0 odyssey_kp_interp also applies cap8 while angle is enabled. The old V1-only wiring did not execute on the van. Both William and public overlays are swept; --plant public overlays public_model onto a frozen full params dictionary. calibrated_pose=None. Live roll comes from vehicleParameters, not the absent liveParameters field.
