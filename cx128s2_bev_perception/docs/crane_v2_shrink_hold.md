# V2 confirmed-Cargo length shrink hold

A current measurement whose observed length is at most 50% of the previous accepted DIRECT box may be a temporarily occluded fragment. For the same confirmed, active Cargo, keep the previous box/size/ID/state for up to 3 sensor seconds. The reference is the preceding accepted observed box, **not the historical maximum locked length**.

The public Cargo is HELD/PREDICTED during this window: direct/beam_detected are false, current public Raw ownership is empty, and bottom/clearance/connection are UNKNOWN. Its coordinates and size are last observations, not newly measured surface. No old Raw or frozen box creates new motion evidence. The actual current fragment length/width/height/center/Raw count remains separately available in internal Output diagnostics. Actual fragment Raw indices are retained separately only while shrink hold is active.

Release early when the observed length recovers to at least 80% of the reference. Accept the current observation after the 3-second deadline if it remains small; repeated small measurements do not restart that window. A gradual reduction that never crosses the trigger ratio does not start hold. A missing frame/body alone never starts it. Missing Cargo after an already-triggered shrink can remain held only until that original deadline. Pure loss retains the existing 0.5s Cargo occlusion / 1.3s LOSS_TIMEOUT policy, and Hook hold is unchanged.

Valid existing GROUND_CONTACT or HOOK_DETACHED completion takes precedence. LOSS_TIMEOUT is deferred only inside an active shrink window. A new accepted Cargo ID, reset, discontinuous stream, or completed cycle clears the window. Rejected zero/duplicate/reversed sensor timestamps do not modify its history. Motion patches are restarted after held geometry; matching formulas are unchanged.

Experimental Node startup parameters (read-only after startup):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `cargo_shrink_hold_sec` | 3.0 | Maximum shrink grace; 0.0 disables it |
| `cargo_shrink_ratio` | 0.5 | Remaining observed length/reference trigger ratio |
| `cargo_shrink_recovery_ratio` | 0.8 | Recovery ratio for early release; must exceed trigger ratio |

Set `CRANE_V2_SHRINK_AUDIT` to a writable CSV path when deliberately running the experimental Node to record current sensor stamp, measured DIRECT/L/W/H/Raw count, displayed DIRECT/HELD/L/W/H, hold elapsed time and state. Existing ROS message keys/state strings, Redis contract and the existing `CRANE_V2_AUDIT` format are unchanged. Startup arguments are supported; a runtime `ros2 param set` is rejected rather than silently changing an unused parameter value. No production YAML/launch selection is changed and the experimental executable is not installed by this work.

Validation artifacts: `/home/sjjin/crane_version_review/v2_shrink_hold_20261103/REPORT.md`. Before-change C4 Working Tree and all pre-existing uncommitted files are in that directory's `baseline_source` with a SHA manifest. Holding a previously incorrect large box can preserve that error for up to 3 seconds; this is an occlusion display policy, not a physical-length ground-truth check or neighboring-object separation fix.
