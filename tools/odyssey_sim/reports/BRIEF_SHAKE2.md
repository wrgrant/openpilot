# Brief: shake simulator, pass 2. Identify the van in stages, then close the loop with noise

Read `BRIEF_SHAKE.md` and `report_shake.md` in this folder first, then `BRIEF_COMMON.md` and the sections of `ODYSSEY.md` they name. Pass 1's code is `tools/odyssey_sim/shake/`; build on it.

## What pass 1 found, and the lead's reading of why it failed

Pass 1 built a physical model (EPS, column with stick-slip, single-track dynamics with tyre relaxation, one-way roll) and both gates failed: open loop 0 of 12 groups inside ±20% gain and ±20° phase, and closed loop S5 reached only 65% of the road's body motion and 59% of its command zero crossings. It still beat the old scalar plant by a wide margin on the training shake window (0.138 against 0.052 m/s² rms, road 0.160).

Three things in the method, not the data, explain much of the failure:

1. **Everything was fitted at once, briefly.** Nine parameters were fitted jointly from command to yaw in one local least-squares run of 25-55 evaluations. Two parameters ended on their bounds, and the fitted "motor lag" came out at 0.25 s when the measured motor responds in 0.024 s (`eps.json`). The chain has measured signals at every joint, so it can be identified one link at a time, and each link then has far more data than the whole chain did.
2. **The closed loop had no noise.** In simulation the stock controller's command crossed zero 0.86 times a second; on the road it crosses 4.07 times. The real loop is driven by things the simulator fed back clean: the 0.1° wheel-angle sensor, yaw-rate and speed noise, road crown, bumps and wind acting on the rack. William's "compressor" mechanism needs those small crossings to exist before a controller can amplify them.
3. **The gates treated poorly excited frequencies as failures.** Stock blocks have little command energy at 1-2 Hz, so their gain and phase there are noise.

Pass 1 concluded that only a new closed-course drive with extra sensors could go further. That is not available: William drives public roads with the comma alone. Exhaust the data in hand first, then say what a short excitation run on a quiet straight road with the existing maneuver suite would add.

## Tasks

### 1. Identify each link separately, with its own held-out check

- **Wheel angle to yaw rate** (single-track model, tyre relaxation, understeer). The EPS is not involved, so use every route: hand-driven ones (1a4, 1a5), the earlier drives, and 9 November. This is the best-excited link in the data. Fit per speed band and check the fitted stiffnesses and yaw inertia are physically consistent across bands. Gate: gain within ±15% and phase within ±15° wherever coherence is at least 0.7, from 0.3 to 3 Hz, on held-out routes.
- **Command to motor torque** (deadband, cap, low-speed fade, lag) from CAN. The measured magnitude has no sign: take the sign from the command while hands are off, and say how you handled reversals.
- **Motor torque to wheel angle** (column inertia, viscous and stick-slip friction, aligning torque). Drive it with the measured motor torque and take the aligning load from the fitted tyre model of the first link, so this fit has only the column's own parameters. Compare Coulomb, Dahl and LuGre friction on held-out windows.
- Use a global search (several starts or a population method) and report profile or bootstrap intervals, so a parameter on a bound or with a flat profile is named as unidentified instead of being reported as a value.
- Roll stays out of the yaw path unless you can show it is needed. Report what the device's own gyro and accelerometer say about roll during the S5 blocks only if the sensor frame can be verified from stationary gravity and known turns.

### 2. Model what excites the loop

- **Sensors:** quantize and add noise to what the controller reads (wheel angle 0.1°, wheel rate, yaw rate, speed), with levels measured from the logs.
- **Disturbance:** estimate the disturbance torque on the rack as the residual of the column link on stock-controller drives (what the measured motor torque and the model cannot explain), fit a simple coloured-noise model to it per speed band, and inject samples of it in closed loop.
- **Plan noise** is already in the logged desired curvature. Say whether the plan must also react to the simulated van at 0.3-3 Hz, using the video simulator's measured planner response (`report_vsim2.md`, `report_plan.md`).

### 3. Gates

Development data: everything up to and including route 1ad. Route 1ae has been seen by pass 1, so it is a check, not proof. **The proof is the next highway drive William makes**: write the forecast for it now, per controller that will run (stock NNLC, with and without the smoothed lane correction), in a locked JSON file: zero crossings per second, command size 0.1 s after a crossing, and the 0.3-3 Hz rms of command, wheel angle and body lateral acceleration, each with an interval.

- **Open loop, whole chain:** as pass 1's gate 1, but score only frequency bins with coherence of at least 0.7 and report the uncovered bins as uncovered.
- **Closed loop:** as pass 1's gate 2, for stock, S3 and S5, with sensor noise and disturbance on. All five statistics within ±25% of the road, S5's peak within 0.3 Hz.
- **The causal test pass 1 could not do:** a factorial in closed loop over model (stock, S5), shaper error gain (1, 2), sensor noise (off, on), disturbance (off, on) and lane correction noise (off, on). Report which factors produce the shake and how they multiply. This is the answer to "what did the earlier simulators miss".

### 4. If the closed-loop gate passes on development data

Carry out task 3 of `BRIEF_SHAKE.md` (William's quiet dead zone with a minimal bump), with one change to the target: William weighs 75 kg and wants single-digit newtons, under once a second and ideally nearer 0.2 times a second, with small reversals welcome only if he cannot feel them. Lock forecasts; do not wire anything into runtime code.

William's refinement of the idea, to evaluate alongside: "a PWM-style bump of 5% or so at various time bases that gives about the same torque as 1-3% would." That is the existing bump mode (`steer_deadband.py`, `bump_comp`: requests under 6% become 6% pulses, on for the request's share of each 0.25 s). Three things to settle in the simulator: the duty has to allow for the dead zone (a 6% pulse delivers motor torque like a linear 3%, because motor torque is about 112 x (command - 0.028), so today's duty under-delivers by about half); the pulse height and time base that move the wheel past stiction with the least felt push; and whether the request must be low-passed first so the pulses follow a real offset and not the 2-4 zero crossings a second of noise.

If it does not pass, specify the shortest excitation run that would close the gap, for a public straight road at 40-60 mph using `openpilot/tools/lateral_maneuvers/odyssey_suite.py` (which holds lane keeping's torque still for 1.5-2 s and adds a pulse): which pulses or sinusoids, how many, how long in total, and which parameter each one pins down. No extra sensors, no closed course.

## Rules

As `BRIEF_SHAKE.md`: repo `~/Code/odyssey-pilot`, branch `odyssey-sim`; never switch branches, commit, push, or touch the comma or the van. Write only in `tools/odyssey_sim/shake/` and `scratchpad/shake/` under the session scratch directory. Another worker is editing `openpilot/selfdrive/ui/`, `openpilot/sunnypilot/selfdrive/controls/`, `openpilot/selfdrive/controls/controlsd.py`, `tools/odyssey_wander.py`, `tools/odyssey_takeovers.py` and `tools/odyssey_drive_map.py`: do not edit or revert those, and note that runtime controller files may change under you (record the git state you ran against). Keep pass 1's results files; write pass 2's beside them with a `2` suffix. Every force in newtons on an 80 kg person beside the m/s², every oscillation as times per second.

## Deliverable

`tools/odyssey_sim/reports/report_shake2.md`: a plain-language answer for William first (does the simulator now reproduce the US-169 shake, and what causes it), then each link's identification with intervals and its held-out check, the noise and disturbance models, the gates, the factorial, the locked forecast for the next drive, and either the bump candidates with locked forecasts or the excitation run specification. Then a short final summary message with each gate's numbers.
