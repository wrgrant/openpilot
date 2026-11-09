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
- `VEHICLE_DYNAMICS.LAT_ACCEL` **is unusable**: uncorrelated with steering-derived lateral accel. Use steering-angle-derived lateral accel.

## Measured facts about this van

From routes 18c–1ab, the 2026-11-09 plant fit (`tools/odyssey_sim/plant/`), and comma's public commaSteeringControl Odyssey data (`~/comma-logs/commaSteeringControl/HONDA_ODYSSEY_2018`, about 250 h).

Steering hardware:
- The EPS motor deadband is about 2.8% command (2-4%). Above it, motor ≈ 112 × (|command| − 0.028).
- Stick-slip friction is much larger: equivalent to 0.29 / 0.20 / 0.14 command at 10 / 20 / 30 m/s.
- Aggregate steering lag is about 0.87 s.
- Gain is 1.66 / 2.45 / 3.73 m/s² per unit command at 10 / 20 / 30 m/s, against 1.03 / 2.09 / 2.27 for the public-data Odyssey.

Behaviour on the road:
- The manual-assist dead zone is about 400-600 driver-torque units.
- Below 15 mph, torque sits at 100% 61-92% of engaged time; the cause is the KP_INTERP boost.
- Curves on 1aa/1ab: the driver took over in about 52%, the van ran 13-29 cm wide mid-curve, and on exit the model's desired lateral accel decays over about 3 s.
- On straights the van sat 8-18 cm right of centre.

NNLC vs stock v0 (open-loop replay, 1a7/1aa/1ab):
- NNLC shows no detectable anticipation lead.
- It reverses torque 2.5 vs 0.73 times/s at 20-35 mph.
- It saturates more below 20 mph.
- A 0.45 s delay raises NNLC reversals by about 30% at 35+ mph.

Closed-loop sim (`tools/odyssey_sim/closedloop/`, 1,400 runs; real V0 / NNLC / `OdysseySteer` on fitted and comma plants):
- Simulated stock tracking cost does not reproduce recorded tracking, so rankings are weak.
- The plants disagree on NNLC vs stock: NNLC is 8% better at 20+ mph on the William plant, tied on public data, worse on comma's generic model.
- Robust across replay and sim: a 0.45 s delay adds 30-39% torque reversals with no tracking benefit (the van is back on the learned delay, `LagdToggle` 1), and the 0.03 kick is worth about 0.5%.
- Low-speed angle steering is unvalidated: no plant is valid below about 10 mph.

The lane-wander simulator overstates wander by 1-3×: use it to reject fragile controllers, not to rank them.

Analysis tools:
- `tools/odyssey_wander.py`: wander and A/B scoring.
- `tools/odyssey_eps_report.py`: held-pulse report.
- `tools/odyssey_lateral_sim.py`: lane-loop sim.
- `tools/odyssey_sim/`, local branch `odyssey-sim`: `replay/` reproduces the van's logged v0/NNLC torque at r ≈ 0.999; `plant/` holds the fitted plant (`plant.py` step API, `plant_model.json`); `closedloop/` uses comma's Controls Challenge harness at `~/Code/ext/controls_challenge`; `nnlc_train/` holds the van-specific model.

## Working on the van (pitfalls)

- `pgrep -f` / `pkill -f` match the calling shell's own command line. Use `pgrep -x <name>` or a `[c]haracter` class.
- openpilot's manager does not restart a process that was killed. To load new UI or controls code, reboot the comma, with the van off.
- Builds on the comma: van off, `PATH=/usr/local/venv/bin:$PATH scons -j3` in `/data/openpilot`. With the van on, `-j8` plus the driving processes runs out of memory. A params-key change rebuilds in about 70 s.
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
13. *Steering characterization drive.* `openpilot/tools/lateral_maneuvers/odyssey_suite.py` holds lane keeping's own torque still for 1.5-2 s at a time and adds one steering torque pulse (0, 3, 4.5, 6 or 8% of max, 0.5 or 1 s, each way), so the wheel's response to the pulse alone shows; holds with no pulse measure the noise. Between holds lane keeping steers normally, so any highway works, gentle curves included. About 4 minutes of maneuvering in runs of 30-35 s, each starting after 2 s at any steady speed from 38 to 80 mph and aborting if you touch the wheel or gas. Finished maneuvers are saved in `LateralManeuverDone`, so it can be done over several drives; it starts over once all are done. Default suite on this branch (`LateralManeuverSuite` = `odyssey`). Setup while parked with the van off, each drive: Settings > Developer > Lateral Maneuver Mode (it clears itself when you turn the van off; over SSH, `echo -n 1 > /data/params/d/LateralManeuverMode` works onroad too). Afterwards: `tools/odyssey_eps_report.py <route dir>` prints the wheel, motor and van response per pulse size against the no-pulse holds, left vs right, and the smallest pulse that turns the wheel beyond the noise. It reads the wheel angle finer than CAN's 0.1° steps by integrating the EPS's rate signal (0.1°/s steps), which moves before the angle ticks over. Lateral-accel requests on top of lane keeping (the first drive's push-pull maneuvers) measure nothing: the model cancels them within about a second.
14. *Updates keep build products.* `system/updated/updated.py` cleans the update with `git clean -dff` instead of `-xdff`, so the first boot after an update rebuilds only what changed instead of the whole tree (a full build took long enough to look like a hung boot). Sunnypilot's Quickboot Mode (Developer, `QuickBootToggle`) writes the `prebuilt` marker at manager start, so later boots skip the build entirely; the update still deletes the marker, so its first boot builds. To install by hand: `git reset --hard <commit>` and `git submodule update --init` in `/data/openpilot`, then `PATH=/usr/local/venv/bin:$PATH scons -j3` with the van off (with the van on, the driving processes plus `-j8` run the 3X out of memory), then `sudo reboot`.
15. *Low-speed angle steering.* `SteerLowSpeedAngle`: below 10 mph, steer on wheel angle (hold 0.02/deg up to 0.35, plus 0.02/deg of angle error), blending to the controller's own command by 18 mph, with the stock KP capped at 8. Code in `low_speed_angle.py`, applied by `OdysseySteer`. `SteerLowSpeedKpCap` alone caps KP without angle steering. Not recommended: stock relies on that gain to steer at all at low speed.
16. *NNLC A/B.* `NnlcAB` rotates NNLC on and off every 60 s (`odyssey_ab.py`, `nnlc.py update_ab`).
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
