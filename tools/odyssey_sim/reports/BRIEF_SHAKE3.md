# Brief: shake simulator, pass 3. Fix the signal timing first, then refit

Read `BRIEF_SHAKE2.md` and `report_shake2.md` in this folder first, then the files they name. Pass 2's code is `tools/odyssey_sim/shake/` (`identify2.py`, `loop2.py`); build on it and write pass 3's files beside it with a `3` suffix.

## The finding that starts this pass

Passes 1 and 2 fitted the van's yaw response to `carControl.angularVelocity`, the yaw rate from the pose filter. On route 1ac segment 7 (S5 on US-169, 0.3-3 Hz band) that signal is **85 ms later than the raw gyroscope**: the raw gyro follows the wheel angle by about 55 ms, the pose-filter yaw by about 140 ms (cross-correlation peaks; the lead's quick check used the single gyro axis best correlated with yaw, r = 0.74, without rotating to the vehicle frame). An unmodelled 85 ms is 31° of phase at 1 Hz and 46° at 1.5 Hz, against a ±15° gate, and a fit with no delay term has to bend the yaw inertia and tyre stiffness to absorb it. That is consistent with pass 2's wheel-to-yaw link passing only 100 of 298 coherent bins and its fitted yaw inertia halving between speed bands.

So before anything else, treat timing and frames as measurements to get right:

1. **Raw gyro in the vehicle frame.** Rotate `gyroscope` (and `accelerometer`) into the vehicle frame with the logged calibration (`liveCalibration`) and verify it: gravity while stationary, sign and size of yaw in known turns against speed × curvature, agreement with the pose-filter yaw at low frequency. Use sensor timestamps. Quantify the pose filter's delay against it per route.
2. **Every other signal's latency.** CAN wheel angle and rate, motor torque and the command each have their own delay to `logMonoTime`. Estimate what you can from the data (for example angle against integrated rate, command against motor torque) and carry each as an explicit, bounded delay in the link it belongs to, reported with an interval, instead of leaving it to be absorbed by physical parameters.
3. **Refit the wheel-angle-to-yaw link** on the corrected signals with the delay explicit. The check is the same as pass 2's (gain ±15%, phase ±15° where coherence is at least 0.7, held-out routes), plus: the fitted stiffnesses and yaw inertia must agree between the two speed bands within their intervals, and the understeer gradient must match the steady-state gain measured in long curves.

If that link now passes, continue down the chain; if it does not, stop there and report exactly what the residual looks like (against frequency, speed, amplitude and route), because nothing downstream can be trusted until it does.

## Then, in order

- **Column link by forward simulation.** Pass 2 fitted the column by inverse force balance on differentiated angles and its forward runs drifted (wheel correlation about 0.75). Fit forward wheel trajectories instead, driven by measured motor torque, with the road load as a slow nuisance term.
- **Closed loop.** Pass 2's S5 simulation was 18 times too large with the rack disturbance on, and pass 2 itself says the disturbance (a model residual) may double-count measurement error. Rebuild the disturbance from the corrected chain, check that with the stock controller the simulated command spectrum matches the road's from 0.3 to 3 Hz, and only then run S3 and S5. Gates as in pass 2.
- **The locked forecast** `forecast_locked2.json` stays as it is. Add a pass-3 forecast for the same next highway drive in a new locked file, for stock NNLC with the bump test's three arms (off, kick 3%, bump 6% pulses; `steer_deadband.py`), since that is what the van will run next.
- If the closed-loop gate passes on development data, do task 4 of `BRIEF_SHAKE2.md` (the quiet dead zone with a pulse-width minimal bump; William weighs 75 kg and wants single-digit newtons, under once a second, ideally nearer 0.2 times a second).

A pulse test is being built by another worker (`BRIEF_PULSE.md`): on straights it freezes lane keeping for 2-3 s and sends one known pulse or pulse train. Its logs will be the clean identification data passes 1 and 2 asked for. Say in your report which parameters still need it and what its report's JSON should contain for you to use it directly.

## Rules

As `BRIEF_SHAKE2.md`: repo `~/Code/odyssey-pilot`, branch `odyssey-sim`; never switch branches, commit, push, or touch the comma or the van. Write only in `tools/odyssey_sim/shake/` and `scratchpad/shake/` under the session scratch directory `/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/`. Another worker is editing `openpilot/` and `tools/odyssey_eps_report.py`: do not edit or revert those. Plots are not stored in git (`shake/.gitignore`): save them under `~/comma-logs/odyssey-evidence/shake_plots/` and link them from the report by absolute path. Tests that need scipy must skip cleanly without it, as `test_identify2.py` does. Every force in newtons on an 80 kg person beside the m/s², every oscillation as times per second.

## Deliverable

`tools/odyssey_sim/reports/report_shake3.md`: a plain-language answer for William first (does the timing fix make the model match, and does the simulator now reproduce the US-169 shake), the measured delays with intervals, each link's check, the closed-loop gates, the locked forecast, and what the pulse test must deliver. Then a short final summary message with each gate's numbers.
