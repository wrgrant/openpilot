# Odyssey branch: sunnypilot + factory longitudinal on Honda Nidec

William's sunnypilot fork for his 2018 Honda Odyssey (Nidec) on a comma 3X: factory ACC for gas and brake while sunnypilot steers, plus Odyssey steering work, tools and UI changes listed below. Start with "What actually runs" before changing any steering code.

## Branches and base

- The van runs `odyssey-agnos20` in `wrgrant/openpilot`: `odyssey-ui` merged with sunnypilot's draft AGNOS 20.3 sync (sunnypilot/sunnypilot#2094 head `41b3de458`, commaai/openpilot `2ddefe2`). opendbc is `wrgrant/opendbc` `odyssey-agnos20`: sunnypilot/opendbc#546 plus the two Honda stock-long commits. AGNOS 20.3.
- `odyssey-stock-long` and `odyssey-ui` are the older AGNOS 19.9 line, on sunnypilot master `932ff06d9`. `odyssey-sim` is local analysis work.
- Tests use the unittest runner `tools/test_runner.py` (upstream dropped pytest); model files come from `lfs.py` (comma's bucket, then sunnypilot's Hugging Face mirror).

## What actually runs (code map)

Lateral path each 10 ms, in order:

1. `modeld` (sunnypilot `modeld_v2`) → `modelV2.action.desiredCurvature`. `CameraOffset` (+ = left) shears the camera input before the model.
2. `controlsd` → `clip_curvature` → `self.LaC.update(...)`. **The class is chosen in `sunnypilot/selfdrive/controls/controlsd_ext.py` `initialize_lateral_control`: with `EnforceTorqueControl` off (the van) it is `sunnypilot/selfdrive/controls/lib/latcontrol_torque_v0.py`, not `selfdrive/controls/lib/latcontrol_torque.py`.** v0: KP 1.0, KI 0.3, FRICTION_THRESHOLD 0.3, KP_INTERP up to 250 at 1 m/s; its setpoint makes stock output nearly independent of the delay. Code added only to `latcontrol_torque.py` never runs on the van.
3. The extension inside either class: `latcontrol_torque_ext.py` → jerk-aware (`LateralJerkTorqueController`, off) → NNLC (`nnlc/nnlc.py`, `NeuralNetworkLateralControl` on, model `neural_network_data/neural_network_lateral_control/HONDA_ODYSSEY.json`, 18 inputs, torque-space PID) and the torque-param override (off).
4. `sunnypilot/selfdrive/controls/lib/odyssey_steer.py` `OdysseySteer.apply`, on the controller's command for any class: low-speed angle steering (`low_speed_angle.py`), then deadband kick/bump (`steer_deadband.py`). The low-speed KP cap goes into both classes via `low_speed_angle.odyssey_kp_interp`.
5. `controlsd` adds `lateralManeuverPlan.torqueOffset` (Lateral Maneuver Mode only) → `actuators.torque`.
6. opendbc Honda `carcontroller.py`: rate limit (full scale in 0.33 s), × STEER_MAX 4096, sign flip → CAN `STEERING_CONTROL` → panda safety → EPS.

Delays: the controller gets `lateralDelay` + `LAT_SMOOTH_SECONDS`. The NNLC extension gets `controlsd_ext.lat_delay` from `get_lat_delay`: the learned value, or, with `LagdToggle` off, CarParams.steerActuatorDelay (0.1) + `LagdToggleDelay`, mirrored in `LagdValueCache`.

Learned torque params: `torqued` + sunnypilot `torqued_ext.py` publish `lateralTorqueParameters` (useParams on for Honda; `LiveTorqueParamsToggle` matters only with `EnforceTorqueControl`). On the van, fully calibrated: latAccelFactor 1.47, friction 0.145, offset −0.055.

A/B tests: `sunnypilot/selfdrive/controls/lib/odyssey_ab.py` rotates arms in 60 s blocks of `time.monotonic()` (= logMonoTime/1e9): `SteerDeadbandCompAB` (off / kick 0.03 / bump 0.06) and `NnlcAB` (on / off, in `nnlc.py`; each switch resets the PID). Score drives with `tools/odyssey_wander.py <route> --ab deadband|nnlc`.

Log names and conventions, as of the AGNOS 20.3 sync:
- liveParameters is logged as **`vehicleParameters`** (roll, angleOffsetDeg, steerRatio, stiffnessFactor).
- Torque params are `lateralTorqueParameters`, delay `lateralDelay`, calibration `extrinsicsCalibration`.
- There is no `livePose` in rlogs; pitch and roll are in `carControl.orientationNED`.
- Read a segment with `LogReader(<path to rlog.zst>)`; a missing path is parsed as a route name.
- `carState.steeringAngleDeg` is opposite to openpilot curvature: a left curve is a negative angle.
- `controlsState.lateralControlState.torqueState.output` and `actuators.torque` have the same sign as the steering angle, opposite to desiredLateralAccel.
- CAN `STEER_TORQUE` is negated again.

CAN signals (DBC `honda_odyssey_exl_2018_generated`):
- `STEERING_CONTROL.STEER_TORQUE`: the command; 4096 = max.
- `STEER_MOTOR_TORQUE.MOTOR_TORQUE`: motor magnitude, 0-256, no sign.
- `STEERING_SENSORS.STEER_ANGLE`: 0.1° steps.
- `STEER_STATUS.STEER_ANGLE_RATE`: 0.1°/s steps, and moves before the angle ticks over. `tools/odyssey_eps_report.py fine_angle()` integrates it.
- `STEER_STATUS.STEER_TORQUE_SENSOR`: the driver's hand torque.
- The pose-filter yaw rate (`carControl.angularVelocity`) is 70-80 ms later than the raw gyroscope on every route (correlation 0.994 on 1ac segment 7): the raw gyro follows the wheel angle by about 70 ms, the pose-filter yaw by about 150 ms. Fine for slow sway and for forces in newtons; for fitting the van's dynamics use the raw gyro in the vehicle frame, or model the delay.
- `VEHICLE_DYNAMICS.LAT_ACCEL` **is unusable**: uncorrelated with steering-derived lateral accel. Use steering-angle-derived lateral accel.

## Measured facts about this van

From routes 18c–1ab, the 2026-11-09 plant fit (`tools/odyssey_sim/plant/`), and comma's public commaSteeringControl Odyssey data (`~/comma-logs/commaSteeringControl/HONDA_ODYSSEY_2018`, about 250 h).

Steering hardware:
- The EPS motor deadband is about 2.8% command (2-4%). Above it, motor ≈ 112 × (|command| − 0.028).
- The EPS caps lane-keeping torque in firmware. Motor torque (`STEER_MOTOR_TORQUE`, raw units) against command, hands off, above 18 mph: 10% → 6, 20% → 14, 30% → 25, 40% → 39, 50% → 50, 60% → 57, 70% → 63, 80% → 67, 100% → 74-76. The top half of the command range adds only a third of the torque.
- Below about 15 mph the EPS gives less at full command: about 15% of that cap under 3 mph, 30% at 3-5 mph, 40-60% from 5 to 15 mph, and all of it from about 17 mph. The same motor puts out a median of 175 and up to 608 units when the driver steers hard by hand under 10 mph, so the hardware has several times more torque than lane keeping may use.
- At full command the van reaches about 0.7 m/s² at 18-30 mph, 1.2 at 30-45 mph and 1.0-1.7 above 45 mph. comma's fleet table (`opendbc/car/torque_data/params.toml`) lists the Odyssey's maximum at 0.84 m/s², against 1.3-2.2 for Toyotas and 1.9-2.7 for Hyundais. While a curve asks for more than 1 m/s², the command is at 100% for 77% of the time at 10-30 mph, 16% at 30-45 mph and 3-4% above 45 mph.
- EPS firmware is `39990-THR-A020` (SuperG SH2A, application at 0xC000, 0x74000 bytes). Its lane-keeping torque table has inputs 0, 128, ... 1024 and outputs 0, 2019, 3230, 3980, 4345, 4577, 4721, 4864, 4864, stored six times. Stock and a community-modified image (table x2.0-2.5, identity `39990-THR,A020`), with decode and diff scripts, are in `~/comma-logs/odyssey-eps/` on the Studio. Nothing has been flashed.
- Nothing points to a fault in this van's steering. Its learned friction (0.145 m/s²) is below comma's fleet value for the Odyssey (0.21) and near Toyota and Hyundai (0.12-0.16). On straight highway it holds its line with a mean command of 0.04-0.05 of max and the wheel about 2° off the sensor's zero, both ordinary for road crown. Self-centring torque is proportional to cornering force, so near straight-ahead every car's wheel stays where friction leaves it; what differs here is that lane-keeping torque below about 3% of max does nothing.
- Stick-slip friction is much larger: equivalent to 0.29 / 0.20 / 0.14 command at 10 / 20 / 30 m/s.
- Aggregate steering lag is about 0.87 s.
- Gain is 1.66 / 2.45 / 3.73 m/s² per unit command at 10 / 20 / 30 m/s, against 1.03 / 2.09 / 2.27 for the public-data Odyssey.

Behaviour on the road:
- The manual-assist dead zone is about 400-600 driver-torque units.
- Below 15 mph, torque sits at 100% 61-92% of engaged time; the cause is the KP_INTERP boost.
- Curves on 1aa/1ab: the driver took over in about 52%, the van ran 13-29 cm wide mid-curve, and on exit the model's desired lateral accel decays over about 3 s.
- On straights the van sat 8-18 cm right of centre.
- The van-trained models act like an audio compressor around zero (William's description, confirmed on straight road above 45 mph). A tenth of a second after the command crosses zero, its size is a median 0.03 of max on earlier drives, 0.08-0.12 on S3 and 0.17-0.19 on S5, against a motor deadband of 0.03. Stock sits inside the deadband 18% of the time, S5 7-10%. The command crosses zero 2.3 times a second on earlier drives and 3.8-4.3 on S5, so every crossing became a push the van answered. The unsmoothed lane correction raised stock's crossings to 4.1 a second.
- How much command swing the van and the driver notice (above 45 mph, 0.5-3 Hz rms, drives 1a7-1ae): a command swing of 0.04 of max moves the wheel 0.13° and the van 0.02 m/s² (stock NNLC, unnoticed); 0.07 moves them 0.27° and 0.045 (stock with the unsmoothed lane correction, not complained about); 0.24 moves them 0.5° and 0.09 (S5, "super jerky"). So fast pulses do get the wheel through the deadband, and the limit of what goes unnoticed is somewhere between 0.045 and 0.09 m/s².
- Why the driver holds the wheel (drives 1ac-1ae, `tools/odyssey_takeovers.py`): 47 presses of 0.3 s or more in 27 minutes of steering above 18 mph. 19 were the driver adding turn in a curve, 12 of them with the command already at 100%; 5 eased a turn; 23 were on straights, where William says his hands were off: the torque sensor reads the wheel's own motion. Example, 1ae segment 4, a right-hand curve at 34 mph: the model asked for 1.2, 1.8, then 2.7 m/s², the command sat at 100% for three seconds and the van gave 0.7-0.85 until it was on the double yellow and the driver took it. Two thirds of raw `steeringPressed` events last under 0.3 s and peak just over the threshold: sensor blips, not hands.
- Low-speed angle steering's first drives (1ac-1ae): time with the command at 100% fell from 72-85% to 13-43% below 10 mph and from 63-78% to 0-7% at 10-18 mph. The wheel was more than 5° short of the request 48% of the time below 10 mph (54% before) and 19% at 10-18 mph (47% before, on different roads). When short with hands off it gets a median 0.5-0.7 command, where the old controller sat at 1.0. Below 10 mph, hands off, the wheel closes a gap at a median 0°/s under 60% command and 8°/s at full command.
- Nearly every curve or turn below 25 mph asking 0.6 m/s² or more ended with the driver's hands on the wheel, before and after low-speed angle steering (14 of 15 on 1ac-1ae; 58 of 58 on 1a7/1aa/1ab).

NNLC vs stock v0 (open-loop replay, 1a7/1aa/1ab):
- NNLC shows no detectable anticipation lead.
- It reverses torque 2.5 vs 0.73 times/s at 20-35 mph.
- It saturates more below 20 mph.
- A 0.45 s delay raises NNLC reversals by about 30% at 35+ mph.

Closed-loop sim (`tools/odyssey_sim/closedloop/`, 1,400 runs; real V0 / NNLC / `OdysseySteer` on fitted and comma plants):
- Simulated stock tracking cost does not reproduce recorded tracking, so rankings are weak.
- The plants disagree on NNLC vs stock: NNLC is 8% better at 20+ mph on the William plant, tied on public data, worse on comma's generic model.
- Robust across replay and sim: a 0.45 s delay adds 30-39% torque reversals with no tracking benefit (the van is back on the learned delay, `LagdToggle` 1), and the 0.03 kick is worth about 0.5%.
- No plant is valid below about 10 mph, so low-speed angle steering is tuned on the road (see its measured facts above).

The lane-wander simulator overstates wander by 1-3×: use it to reject fragile controllers, not to rank them.

Physical shake simulator (`tools/odyssey_sim/shake/`, three passes, `report_shake*.md`): not validated; don't tune with it. With timing corrected, a single-track model predicts yaw from wheel angle with a median gain within about 10% up to 1.4 times a second, but individual windows scatter widely (gain 0.38-1.53) and at wheel swings under 0.15° the van yaws nearly twice what the measured angle implies: ordinary driving puts the steering input at the 0.1° sensor's resolution. The command-to-wheel link has never passed a forward check. It needs known inputs, which the pulse test (`BRIEF_PULSE.md`) provides.

Analysis tools:
- `tools/odyssey_wander.py`: wander and A/B scoring.
- `tools/odyssey_eps_report.py`: held-pulse report.
- `tools/odyssey_lateral_sim.py`: lane-loop sim.
- `tools/odyssey_sim/`, local branch `odyssey-sim`: `replay/` reproduces the van's logged v0/NNLC torque at r ≈ 0.999; `plant/` holds the fitted plant (`plant.py` step API, `plant_model.json`); `closedloop/` uses comma's Controls Challenge harness at `~/Code/ext/controls_challenge`; `nnlc_train/` holds the van-specific model. See [the NNLC experiment guide](tools/odyssey_sim/nnlc_train/README.md) for route splits, the default-off model override, and the limits of offline improvements.

## After every drive

- Ask William what he noticed and where before scoring anything. He feels what the metrics filter out, and a mismatch between his report and a metric is a fault in the metric.
- Report sideways motion in units he can feel: the push on an 80 kg person in newtons (80 × the typical peak of speed × yaw rate at 0.3-3 Hz, over 5 s) and how many times a second it changes direction. Earlier drives are ±7-9 N at about 0.3 Hz. S5 on US-169 was ±20 N typical, ±40-50 N at worst, 2.7 direction changes a second, with 1.1 m/s³ rms of jerk.
- William's target (he weighs 75 kg, so forces quoted for 80 kg read 6% high for him): single-digit newtons, under once a second and ideally nearer 0.2 times a second. Small reversals that keep the van straight are welcome as long as he cannot feel them.
- Map the drive by test arm, push, speed band and what the scorer counted (the 2026-11-09 map is `~/comma-logs/odyssey-evidence/maps/`), so he can match stretches of road to what he felt.

## Working on the van (pitfalls)

- Run the UI render tests (`openpilot/selfdrive/ui/tests/test_odyssey_ab_panel.py`, `.../developer_ui/tests`) in their own `test_runner.py` call: run in parallel with each other they fail on a shared messaging socket.
- `pgrep -f` / `pkill -f` match the calling shell's own command line. Use `pgrep -x <name>` or a `[c]haracter` class.
- openpilot's manager does not restart a process that was killed. To load new UI or controls code, reboot the comma, with the van off.
- Builds on the comma: van off, `PATH=/usr/local/venv/bin:$PATH scons -j3` in `/data/openpilot`. With the van on, `-j8` plus the driving processes runs out of memory. A params-key change rebuilds in about 70 s.
- Before installing a change to controlsd or the lateral controller, run `tools/odyssey_sim/replay/controlsd_check.py <segment rlog.zst> '<params json>'`. It runs the real controlsd process over a recorded minute with the van's car parameters and fails if it dies or puts out a non-finite command. Unit tests and an import check do not exercise the controlsd loop. openpilot's stock process replay builds the Odyssey with a PID controller, so use this script, which takes CarParams from the log.
- Pushing from the Studio: the LFS pre-push hook (`lfs.py pre-push`) hangs, so push with `--no-verify`. Pushing workflow files needs the `workflow` gh scope.
- Reach the comma at `ssh comma` (home Wi-Fi, 192.168.1.93) or `ssh -i ~/.ssh/id_ed25519_comma comma@100.66.59.37` over Tailscale. `DisableUpdates` is 1 while testing `odyssey-agnos20`.

## The carried patch

Two repos, five commits in total.

**`wrgrant/opendbc`, branch `odyssey-stock-long`** (on top of the sunnypilot/opendbc pin `aa9bff6f`):

1. *Honda Nidec: stock longitudinal safety mode.* `opendbc/safety/modes/honda.h` gets a new SP safety bit, `HondaSafetyFlagsSP.STOCK_LONGITUDINAL = 4`. With it set:
   - The TX allow-list is steering (0xE4/0x194), LKAS_HUD (0x33D), and SCM_BUTTONS (0x1A6/0x296) on bus 0. BRAKE_COMMAND (0x1FA) and ACC_HUD (0x30C, which carries PCM gas and speed) are not allowed.
   - The stock ACC module's own BRAKE_COMMAND and ACC_HUD are always forwarded.
   - SCM_BUTTONS is cancel-only while controls are not allowed.
   - If the gas interceptor bit is also set, the interceptor wins.
   - Tests are in `opendbc/safety/tests/test_honda.py` (`TestHondaNidecStockLong*`).
2. *Honda Nidec: HondaEnforceStockLongitudinal param.* `opendbc/sunnypilot/car/interfaces.py::_initialize_honda` reads the param. On a Nidec with no interceptor it sets `HondaFlagsSP.STOCK_LONGITUDINAL = 16` and the safety bit, and sets `openpilotLongitudinalControl=False` and `pcmCruise=True`. The carcontroller then sends no brake or ACC_HUD, and it cancels the stock ACC through SCM_BUTTONS. BOSCH_SUPPLEMENTAL_1 is now sent only on Bosch. Tests are in `opendbc/sunnypilot/car/honda/test_honda.py`.

**`wrgrant/openpilot`, branch `odyssey-stock-long`:**

3. The opendbc submodule points at `wrgrant/opendbc` `odyssey-stock-long`.
4. The `HondaEnforceStockLongitudinal` param key goes in `openpilot/common/params_keys.h` and the car-interface param list (`openpilot/sunnypilot/selfdrive/car/interfaces.py`). There is also a Vehicle → Honda toggle, "Nidec: Enforce Factory Longitudinal Control" (`openpilot/selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/honda.py`). It can only be changed offroad, it has a confirm dialog, it is disabled on Bosch, and it uses `gui_app.push_widget`.
5. This file.

**`wrgrant/openpilot`, branch `odyssey-ui`** (on top of `odyssey-stock-long`), UI additions:

6. *Lead chevron with factory longitudinal.* `openpilot/selfdrive/ui/onroad/model_renderer.py` draws the lead chevron and Chevron Info (distance, speed, time-to-collision) whenever `radarState` is valid. Upstream only draws them when openpilot does longitudinal.
7. *Side panel and Steering Arc.* The right-hand developer column (`DevUIInfo`) shows CURVE SPD, STEER TRQ (commanded torque as a % of the car's max, the same signal the Steering Arc draws; blank while the driver is steering), steering angle, and desired/actual lateral accel. Each row puts the label and its unit upright on one line, and the value is right-aligned with fixed-width digits so it doesn't jitter. Lead distance and speed moved to the chevron. The Steering Arc (`TorqueBar`) is on by default, keeps one thickness at every torque, and has no centre dot. It reads like a VU meter: square-seamed white (0–60% of the end stop), yellow (60–85%) and red (85–100%) zones, with thin square seams between zones, rounded ends only at the two outer stops, the unlit zones shown faintly so you can see where each starts, and the fill grows through each zone in that zone's colour (`meter=True`); the fill keeps the rounded shape of the outer ends, and a red outline goes around the whole bar while commanded torque is pinned at 100%, the state that leads to the take-over alert. Code: `openpilot/selfdrive/ui/sunnypilot/onroad/developer_ui/`, `openpilot/selfdrive/ui/mici/onroad/torque_bar.py` (`fixed_height`, `center_dot`). Tests: `developer_ui/tests/test_odyssey_elements.py`.
8. *UI replay.* `tools/odyssey_ui_replay.py` plays a recorded drive (rlog + qcamera) through the real onroad UI on a Mac and records an MP4, so UI changes can be checked without the device. Usage is in the file's docstring.
9. *Green light chime delay.* `GreenLightAlertDelay` (Visuals → Green Light Chime Delay: Off, 0.5 s, 1 s, 1.5 s; default 0.5 s) holds the Green Traffic Light Alert chime for that long after the light turns green, and skips it if the driver lifts off the brake, presses the gas or moves in that time. With Brake Hold (foot already off the pedal) it chimes after the delay unless the van moves. Code: `sunnypilot/selfdrive/controls/lib/e2e_alerts_helper.py`. Tests: `sunnypilot/selfdrive/controls/lib/tests/test_green_light_delay.py`.
10. *Wander meter.* `tools/odyssey_wander.py <route_dir>` finds stretches where openpilot steers alone and measures the slow 7-20 s lane ping-pong: lane-position swing, its period, how much the requested lateral accel swings, and how closely the actual follows. A low follow ratio means the swing is in what openpilot asks for (model/planner, camera offset, steer delay), not the controller or the steering. Needs daytime highway drives for the lane-position column.
11. *Steering deadband kick.* Applied by `OdysseySteer` (see the code map). `SteerDeadbandComp` (normalized torque, default 0 = off) adds that much torque in the direction of any steering request, blended in over 0.01 around zero. The Odyssey's EPS motor barely answers commands below about 2-4% of max torque, and openpilot's friction term only fires on an error the 0.1° angle sensor can see, so the small corrections that hold the lane don't happen. Code: `sunnypilot/selfdrive/controls/lib/steer_deadband.py`, applied after NNLC in `latcontrol_torque.py`. With `SteerDeadbandCompAB` on, the kick runs only in every other 180 s block of the monotonic clock, so one drive compares kick and no kick on the same roads; `tools/odyssey_wander.py <route dir> --ab` scores it. Set on the device with `echo -n 0.03 > /data/params/d/SteerDeadbandComp` and `echo -n 1 > /data/params/d/SteerDeadbandCompAB` (re-read every 5 s).
12. *Lateral simulator.* `tools/odyssey_lateral_sim.py --road <route dirs>` drives the van in closed loop on the curves recorded on real highway drives. It models openpilot's request as road + 0.34·offset + 0.91·sideways speed (fitted from routes 18c and 19c, R² 0.98), the torque controller, and the steering: motor deadband, friction, lag, a one-sided pull, and the 0.1° sensor. It prints the same numbers as the wander meter, compares ideal steering, this van, and deadband-kick settings, and its steering values come from the characterization drive (item 13). Normal driving can't pin those values down: friction 0.02-0.08 and lag 0.25-0.6 s all fit routes 18c and 19c. Under every fit, a 0.03-0.037 kick cuts the 7-20 s wander from about 7.9 to 6.1-6.4 cm (with perfect steering it would be about 4.1 cm), and above 0.05 the torque starts to buzz.
13. *Pulse test (steering characterization).* Measures what a small steering pulse does, with lane keeping out of the way. Selected as experiment `pulse` (Settings > Vehicle > Honda > Odyssey A/B experiment, parked); every drive starts paused and a tap on the panel starts or pauses it. `odyssey_pulse.py` runs at 100 Hz in controlsd after `OdysseySteer`: it waits for a straight (the model's next 3 s under 0.25 m/s², both inner lines at 0.7 or more, lane at least 3.2 m wide, the van within 0.25 m of centre with room to drift, low-passed heading under 0.003 rad, wheel still, hands off, lane keeping's mean command over the last second under 6%), then takes the command for one shot of 1.15-2.1 s: zero for 0.3 s, one pulse or pulse train, zero for 0.8 s. Zero is what bump mode sends between pulses. Lane keeping then steers until the van is settled again. A shot ends at once, with lane keeping back on that frame, if the van has moved 0.20 m, its side is within 0.30 m of a line, a curve or lane change arrives, the lines fade, speed changes, the driver's torque passes 1200 units or stays above 800 for 0.15 s, the blinker or brake comes on, or any input goes stale; an aborted shot is retried later. Any fault in the test hands steering to lane keeping and switches the test off until restart. Plan: pulses of 2-12% for 0.05-0.4 s, trains of 5-8% on for 20-60% of 0.25 or 0.5 s periods for 1 s, and one sham in ten, each direction, 112 shots a pass, three passes per speed band (25-45 and 45-70 mph), progress kept in `OdysseyPulseProgress` (reset in settings). Each shot is logged as `odyssey_pulse` records. `tools/odyssey_eps_report.py <route dirs> -o out.json` turns the logs into wheel movement, motor answer and push on the driver per pulse against the shams. Replayed over five recorded drives the entry rules give about 1.4 completed shots a minute of eligible steering (2 on a good highway drive), so a pass takes 55-80 minutes of highway; lane keeping is off 3-6% of that time. On the fitted plant, starting at the entry limits with a 0.15 m/s² sideways bias and a pulse at the 15% ceiling, the van's side stays 29 cm or more from the line. `tools/odyssey_sim/replay/controlsd_check.py <straight minute> --pulse` proves the wiring: paused equals off, running sends shots. It replaced the held-pulse maneuver suite, whose pulses lane keeping buried.
14. *Updates keep build products.* `system/updated/updated.py` cleans the update with `git clean -dff` instead of `-xdff`, so the first boot after an update rebuilds only what changed instead of the whole tree (a full build took long enough to look like a hung boot). Sunnypilot's Quickboot Mode (Developer, `QuickBootToggle`) writes the `prebuilt` marker at manager start, so later boots skip the build entirely; the update still deletes the marker, so its first boot builds. To install by hand: `git reset --hard <commit>` and `git submodule update --init` in `/data/openpilot`, then `PATH=/usr/local/venv/bin:$PATH scons -j3` with the van off (with the van on, the driving processes plus `-j8` run the 3X out of memory), then `sudo reboot`.
15. *Low-speed angle steering.* `SteerLowSpeedAngle`: below 10 mph, steer on wheel angle (hold 0.02/deg up to 0.35, plus 0.02/deg of angle error), blending to the controller's own command by 18 mph, with the stock KP capped at 8. Code in `low_speed_angle.py`, applied by `OdysseySteer`. `SteerLowSpeedAngleAB` alternates the error gain each 60 s between 0.02/deg ("base") and 0.08/deg ("firm", full torque when 12° short); `tools/odyssey_takeovers.py` splits its low-speed numbers by arm. `SteerLowSpeedKpCap` alone caps KP without angle steering. Not recommended: stock relies on that gain to steer at all at low speed.
16. *NNLC A/B.* `NnlcAB` rotates NNLC on and off every 60 s (`odyssey_ab.py`, `nnlc.py update_ab`).
19. *Van-trained NNLC models.* `nnlc/odyssey_models/HONDA_ODYSSEY_WILLIAM_{V1,S3,S5}.json`, trained on this van's drives (`tools/odyssey_sim/nnlc_smooth/`). In the calibrated simulator, with the logged plan held fixed, S3 and S5 track about 3x better than stock NNLC and cut the best-fit lag between plan and response from 0.47 s to 0.13-0.17 s. **S5 failed on the road** (drives 1ac-1ae): on US-169 at 60 mph it pushed the driver side to side with ±20 N typical and ±40-50 N at worst, 1.3 times a second (earlier drives: ±7-9 N), and the 7-20 s weave was no smaller. William: "comically bad, almost nauseous". Don't run S5. S3 is undecided: with the lane correction off it was as calm as stock (±5 N over 3.8 minutes, mostly near 50 mph), and with the unsmoothed lane correction on it shook like S5 (±15 N). `NnlcModelAB` is off on the van. `NnlcModelOverride` (file name) runs one model; `NnlcModelAB` rotates stock / S3 / S5 every 180 s, and `tools/odyssey_wander.py <route dir> --ab nnlc_model` scores it (skipping each block's first 30 s). Each model JSON's `odyssey_smoothing` block drives `NnlcResponseShaper` in `nnlc.py`. A model that fails validation (18 inputs, finite outputs) falls back to stock. All trained candidates, including V2 and S4, which were not shipped, are kept outside git in `~/comma-logs/odyssey-nnlc-models/` on the Studio. The older `tools/odyssey_sim/nnlc_train` and `calib` scripts still look for them in the `neural_network_data` submodule, so pass that folder instead.
20. *Video simulator.* `tools/odyssey_sim/vsim/` runs the van's actual driving model (`driving_supercombo.onnx`, ONNX Runtime) on recorded road and wide camera video, warped to a simulated pose, in closed loop with the real V0/NNLC controller and a perturbation plant around the logged path. Open-loop it matches the logged desired curvature at r=0.9998, at about 30 fps. Held-out closure and controller rankings are not yet validated, and its locked forecast of S5's body jerk (+0.04 m/s³ over stock) was a tenth of the road result (+0.34). Video lives in `~/comma-logs/04b44e4c4fe44232_<route>/` (fcamera.hevc, ecamera.hevc); the comma keeps it only until its disk fills. Research reports and briefs: `tools/odyssey_sim/reports/`.
21. *Lane-centring correction.* `OdysseyLaneCenter` (always on) or `OdysseyLaneCenterAB` (off/on every 180 s) adds `-0.3·offset - 1.2·speed·heading` m/s², capped at ±0.15, to the model's desired curvature in controlsd (`odyssey_lane.py`). Offset and heading come from a cubic fit to the centre of the two inner lane lines over the first 25 m. It is low-passed at 0.5 s, because the lane heading is noisy frame to frame: unsmoothed (drives 1ac-1ae) it carried as much 0.5-3 Hz content as 7-20 s content and doubled the fast swing of the request. It runs only above 25 mph, with no lane change or blinker and a model frame under 0.2 s old, and fades to zero when either line is under 50% likely. Model frame: y is positive to the van's right; the lane centre moves toward +y with the left blinker on. In the video simulator it cut held-out wander by 1.7 cm RMS (`tools/odyssey_sim/reports/report_plan.md`); not yet road-proven. Score with `tools/odyssey_wander.py <route dir> --ab lane`. With `NnlcModelAB` also on, block k runs model arm k % 3 and lane arm k % 2, so six blocks cover every combination.
22. *CameraOffset does nothing on the default model.* Only sunnypilot's `modeld_v2` (custom model bundles) and the UI read it. The van runs upstream `selfdrive/modeld` because no bundle is selected, so the +0.12 set to counter the rightward bias never reached the steering. The lane-centring correction is what acts on that bias now.
23. *Scoring road A/B tests.* `tools/odyssey_wander.py <route dir> [<route dir> ...] --ab lane|nnlc_model|wander|nnlc|deadband` pools drives. It reads each drive's initData to see which A/B params were on, skips drives where the requested one was off, and with `OdysseyLaneCenterAB` and `NnlcModelAB` both on scores the models on lane-correction-off blocks only (the locked forecast's condition). Per arm it reports wander, sway (body lateral accel in the 7-20 s band, independent of the lane lines), shake and how often the driver pressed the wheel. Shake is the body's lateral accel swing at 0.5-3 Hz over all steering time, wheel presses included; read it first. Wander is scored only on 45 s stretches with no wheel press, and a controller that shakes the wheel trips the press sensor, so the arm that shakes goes unscored: on 1ac-1ae that hid S3 and S5 behind "too little data". Shake's null spread on eight earlier drives is ±0.007 m/s² around 0.035; S5 scored +0.06 above stock above 45 mph. One drive cannot settle a test: scored as a null test on five drives without any lane A/B (`--force`; 30 blocks, 42 scored minutes), the two arms differed by -0.55 cm with a 95% interval of [-2.7, +1.6] cm, and sway differed by 0.05 m/s² from road curvature alone. A 2 cm wander effect needs roughly 9 hours of mixed driving, fewer on highway. Baseline on those drives: about 9.5 wheel presses per 10 minutes of openpilot steering above 25 mph.
24. *EPS torque mod, prepared and not flashed.* `tools/odyssey_eps/rwd.py` decodes and rebuilds `39990-THR-A020` images; it reproduces the community-modified image from stock bit for bit. Candidate images with the torque table at x1.5 and x2.0 are in `~/comma-logs/odyssey-eps/images/`, and a reviewed flashing kit in `~/comma-logs/odyssey-eps/kit/`. Honda's binaries stay out of git. A modified EPS reports `39990-THR,A020`; opendbc then routes the Odyssey's steering command through `OdysseyEpsMap` (`opendbc/sunnypilot/car/honda/eps_mod.py`, param `OdysseyEpsMap`), which inverts the firmware table so the default drives like stock and extra authority is raised step by step. The table output is a virtual hand torque of about 2550 sensor units at stock full command, fed to the normal assist curve. Hand steering in the logs covers that curve only to about 2800 units, so torque beyond 1.1x stock's virtual torque is an extrapolation until measured. `tools/odyssey_eps/README.md` has the map and the measurement tool; `tools/odyssey_sim/reports/report_eps.md` has the flasher review and the step-up plan.
25. *Takeover report.* `tools/odyssey_takeovers.py <route dir> [<route dir> ...] [-v]` lists every real wheel press while openpilot steered above 18 mph and sorts it by cause: in a curve with the command already at 100% (the EPS is out of torque), in a curve with torque to spare, against the turn, or on a straight. Below 25 mph it reports how often the wheel was more than 5° short of the request and what command it was getting then, by `SteerLowSpeedAngleAB` arm when that ran. It is the outcome measure for hands-off driving and the before/after measure for the EPS torque mod. It caches each route's table as `takeovers.npz` in the route folder.
26. *Driver-controlled A/B tests.* Settings > Vehicle > Honda > "Odyssey A/B experiment" (parked) picks one experiment: `lane` (off / on), `bump` (off / kick / bump), `nnlc_model` (stock / S3) or `lowspeed` (base / firm); "Driver selects A/B arm" makes it manual. On the road a panel on the left shows the arm that controls reports back in large type, whether the current second counts and why not, and straight and curve seconds per arm against targets of 300 and 60. A tap on the panel switches to the next arm (manual mode only); with manual off the arms rotate by the clock and the panel only reports. One definition, `odyssey_ab.good_data`, decides what counts, live and offline: steering on, no blinker or lane change, in the speed range, 5 s after a switch, no wheel press of 1 s or more, lane lines visible where the score needs them. `LiveAB` in controlsd counts and writes an `odyssey_ab` record to the log every second (a `logMessage`), so the scorers read the arm from the log and fall back to the clock for older drives. Params: `OdysseyABExperiment` (empty = off, and then nothing here changes steering), `OdysseyABManual`, `OdysseyABArm`, `OdysseyABTotals`, `OdysseyABReset`; the live status is in memory params (`/dev/shm/params`, `OdysseyABStatus`). In manual mode the older clock flags (`SteerDeadbandCompAB`, `SteerLowSpeedAngleAB`, `OdysseyLaneCenterAB`, `NnlcModelAB`) are ignored and every other feature takes its always-on setting. Totals are reset in settings.
27. *Drive map and road-matched scoring.* `tools/odyssey_drive_map.py <route dir> [...] -o map.html` draws the drives on a street map coloured by test arm, sideways push in newtons, speed band, or what counted and why not, with a per-arm table. `tools/odyssey_wander.py <route dirs> --ab <test> --road-matched` compares arms on the same piece of road (50 m cells, same heading and speed band) and bootstraps over drive pairs; its null on eight earlier drives is +3.6 N with an interval of [-0.5, +8.0] N from four drive pairs, so it needs the repeated passes manual mode makes possible.
17. *Tailscale on the comma.* `tools/odyssey_tailscale.sh install|start|stop|status`, binaries and state in `/data/tailscale`, userspace networking, started from `launch_chffrplus.sh`. Host `odyssey-comma`.
18. *Joke screensaver.* `system/ui/sunnypilot/widgets/screen_saver.py` and `jokes.py` replace the bouncing logo. The ScreenSaverEnabled toggle and timeout still apply.

The param key matches mvl-boston's fork, so a device that already has it set carries the setting over.

## Rebasing onto a new sunnypilot release

```bash
cd ~/Code/odyssey-pilot
git fetch origin                       # sunnypilot/sunnypilot
NEW=<new sunnypilot commit or tag>     # e.g. the next staging/... tag or release-tizi's master commit
git checkout -b odyssey-stock-long-<version> $NEW
git submodule update --init opendbc_repo

# opendbc: rebase the two patch commits onto the new pin
cd opendbc_repo
git fetch wrgrant odyssey-stock-long
git checkout -b odyssey-stock-long-<version>
git cherry-pick dc1156da 3acb7c30      # or the current tips of wrgrant/opendbc odyssey-stock-long
source setup.sh && (cd opendbc/safety/tests && bash test.sh) && python -m unittest opendbc.sunnypilot.car.honda.test_honda
git push wrgrant HEAD
cd ..

# openpilot: point the submodule at the fork and re-apply the param/UI commit
git config -f .gitmodules submodule.opendbc.url https://github.com/wrgrant/opendbc.git
git add .gitmodules opendbc_repo
git commit -m "opendbc: wrgrant/opendbc odyssey-stock-long"
git cherry-pick <param/UI commit> <ODYSSEY.md commit>
git push wrgrant HEAD:odyssey-stock-long
```

Conflicts to expect: `honda.h` TX lists and `_initialize_*` ordering in `opendbc/sunnypilot/car/interfaces.py`. Before pushing, check that the catalog the new base reads (`openpilot/sunnypilot/models/fetcher.py`, `MODEL_URL`) still lists TSFDOM with a `minimum_selector_version` equal to `REQUIRED_JSON_VERSION` in `openpilot/sunnypilot/models/helpers.py`.

## Install

On the device (parked, van off), use the installer URL `installer.comma.ai/wrgrant/odyssey-stock-long`, or switch the git remote in `/data/openpilot`. The device builds from source on first boot.
