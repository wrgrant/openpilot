# Odyssey steering research reports

Worker reports and briefs from the simulator and controller research. Read `BRIEF_COMMON.md` first: it holds the goals, facts and rules every worker shares. The large evidence files (JSON, NPZ caches) are not kept here. The reports name a session scratch directory under `/private/tmp`; its contents are copied to `~/comma-logs/odyssey-evidence/` on the Studio, minus virtualenvs and downloaded driving models.

| Report | What it covers |
|---|---|
| report_plant, report_calib, report_calib3 | Steering plant fits, the yaw-validated plant v2, and the failed attempts to close the lane loop with a fitted planner |
| report_nnlc, report_closedloop, report_van_nnlc, report_nnlc_smooth | NNLC replay, closed-loop comparison, and the van-trained models V1/V2/S1-S5 |
| report_perception | The post-EPS roughness metric that replaced raw torque-reversal counts |
| report_wander, report_curves, report_lowspeed | Wander friction assistance, curve look-ahead, and low-speed angle steering |
| report_vsim, report_vsim2 | The video simulator: the real driving model on reprojected recorded video, in closed loop |
