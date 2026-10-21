##  Don't push this file to main. For this branch only

## Use markdown editor to easily check boxes and update tables in this file

# Shooter testing: pose-based aiming

Testing checklist for the pose-based shooting changes (see `SHOOTING-USING-POSE-PLAN.md`). Work top to bottom: later sections assume the earlier ones pass.

**What changed:** LT now aims at the hub center and picks RPM/angle/boost from the swerve pose (odometry + MegaTag2), not from the Limelight. The shot table (`ShooterConstants.rpmAngle`) is unchanged: its keys are the floor distance from the robot to the hub center, the same as the 2026 code measured.

## Reference

| Item | Value |
| --- | --- |
| Blue hub center | (4.6256, 4.0346) m, blue-origin field |
| Red hub center | (11.9154, 4.0346) m |
| Table keys | 0.0 to 4.4 m in 0.2 m steps; distances outside are clamped |
| Aligned tolerance | 2° (`AlignAndMove.ALIGN_TOLERANCE_DEG`) |
| "In range" for rumble/dashboard | 0 < distance ≤ 4.0 m (`MAX_SHOOT_DISTANCE_M`) |
| Heading-slip threshold | 3° (`Swerve.HEADING_SLIP_WARNING_DEG`), LED debounce 0.25 s |

| Telemetry key | What it shows |
| --- | --- |
| `Shooter/ForwardDistance` | Hub distance, **rounded** to the table key |
| `Shooter/RawHubDistance` | Hub distance, unrounded (m) |
| `Shooter/HubHeadingErrorDeg` | Aim heading minus robot heading (°); near 0 when aimed |
| `Shooter/Aligned` | Facing the hub within 2° |
| `Vision/HeadingSlipWarning` | Heading-slip warning (not on the Elastic layout yet) |
| `Drive/MT1 Heading Slip Deg` | Last MegaTag1-vs-pose heading difference |
| `Drive/Heading Slip Warning` | Same warning, from Swerve |
| `Drive/where the bot think it is swerve x / y / degree` | Pose |
| `Drive/Field 2d`, `Drive/megatag2` | Pose and latest MegaTag2 pose on a field view |

## 1. Pose accuracy and aiming (do this before any shots)

- [ ] **Limelight settings left over from the old system.** The old aiming used per-tag pipelines (1–6) with 3D offsets pointing at the hub center. Nothing needs those now. Check they can't throw the pose off:
  - [ ] In the Limelight web UI, check pipeline 0 (the one the code uses now): 3D offset / Point of Interest is zero, "Use Map POI Offsets" is off, and there's no tag ID filter hiding hub or field tags.
  - [ ] Check the field map is the stock 2026 map (welded or AndyMark, matching your field), not an old custom one with moved tags.
  - [ ] Check the camera pose in the UI matches `Cameras.java` (0.014 m forward, 0 left, 0.516 m up, pitch −55°). The code sends these, but a mismatch in the UI is worth catching.
  - [ ] Delete or ignore pipelines 1–6; the code no longer switches to them.
  - [ ] **Pose check:** place the robot at 3 tape-measured spots and compare `Drive/megatag2` and the MegaTag1 pose against the real position. They should agree within a few cm and 1–2°. A consistent offset (always shifted the same way) suggests a leftover offset, wrong field map or wrong camera pose.
  - Per the Limelight docs, the 3D offset / POI only moves `tx`/`ty` (aiming) and `tdist`, not MegaTag localization. So this is most likely fine, and the firmware update probably wiped the old offsets anyway. It's still worth confirming, because a bad pose now breaks both aim and distance.
- [ ] Power on, press Y facing away from your driver station, and drive near the hub. Once tags are seen, the pose on Field2d matches where the robot really is.
- [ ] **Teleop without auto:** enable teleop straight away (no auto run). The pose starts at (0, 0) until MegaTag2 sees tags. Confirm it snaps to the right place once tags are in view, and note how long that takes. Shooting before then uses a wrong distance.
- [ ] **Distance check:** tape-measure robot-center to hub-center at 1.5, 2.5 and 3.5 m. `Shooter/RawHubDistance` should be within about 5 cm, and `ForwardDistance` should show the matching key.

| Measured (m) | `RawHubDistance` (m) | `ForwardDistance` | Difference |
| --- | --- | --- | --- |
| 1.5 | | | |
| 2.5 | | | |
| 3.5 | | | |

- [ ] **Aim check:** hold LT from at least 4 positions around the hub. The robot points at the hub center (not a tag face), `Shooter/HubHeadingErrorDeg` settles within ±2°, `Shooter/Aligned` turns true, the LEDs go green, and rumble fires.
- [ ] **Both alliances:** repeat the aim check on red. Include a start facing directly away from the hub, so the turn crosses ±180°. The robot turns the short way and doesn't spin a full circle.
- [ ] Hold LT with the sticks idle: once aligned, the wheels lock (X pattern) and stay locked with no twitching.
- [ ] Hold LT while driving: the robot keeps turning toward the hub while it translates, with no fighting or stuttering in the modules.
- [ ] **Camera blocked:** cover the Limelight, drive a short loop, hold LT. It still aims at the hub, using odometry only.
- [ ] Odometry drift: with the camera covered, drive for 10, 20 and 30 s, then uncover it and see how far the pose jumps. That jump is how far off aim and distance were.

## 2. Shooting from the table's recorded points

Goal: confirm the 2026 table carries over unchanged. Shoot from the table's own keys.

- [ ] Place the robot at each distance below (tape-measured from hub center, facing the hub), check `ForwardDistance` reads that key, then hold LT and shoot 5 balls.

| Key (m) | RPM | Angle | Boost | Made / 5 | Miss pattern (short / long / left / right) |
| --- | --- | --- | --- | --- | --- |
| 0.4 | 1900 | 0.0 | 1.15 | | |
| 0.8 | 1900 | 0.0 | 1.19 | | |
| 1.2 | 2040 | 5.85 | 1.21 | | |
| 1.6 | 2040 | 8.0 | 1.23 | | |
| 2.0 | 1980 | 12.61 | 1.25 | | |
| 2.4 | 2020 | 14.10 | 1.27 | | |
| 2.8 | 2060 | 16.47 | 1.29 | | |
| 3.2 | 2100 | 18.30 | 1.31 | | |
| 3.6 | 2140 | 20.45 | 1.33 | | |
| 4.0 | 2200 | 22.5 | 1.30 | | |
| 4.4 | 2240 | 24.5 | 1.32 | | |

- [ ] Repeat a few points from different angles around the hub (left, center, right). The distance is to hub center, so makes shouldn't depend on which side you shoot from.
- [ ] Repeat a few points on the **red** alliance.
- [ ] If one direction misses consistently (always long, say), check the distance (section 1) before changing the table.

## 3. First-shot boost and jitter

- [ ] Hold LT, then RT: the first ball gets the boost (RPM above target), then the RPM drops to normal after the dip.
- [ ] **Boost once per press:** while still holding LT, drive slowly across a key boundary (e.g. 2.3 → 2.5 m). RPM and angle change to the new key, but there's **no** second boost.
- [ ] Release LT, press again: the boost happens again.
- [ ] Passes (RB mid, B far): every press gets one boost, including pressing the same pass twice in a row.
- [ ] **Jitter:** park on a key boundary (e.g. `RawHubDistance` ≈ 2.30 m) and hold LT. If `ForwardDistance` flickers between keys and the flywheel or hood hunts, the per-loop lookup is the cause. **First fix:** move the lookup in `HashShoot.execute()` back into `initialize()`. Interpolation (section 5) is the longer-term fix.

## 4. Heading-slip warning (gyro off by more than 3°)

- [ ] Normal driving with tags in view: no pink. Record the usual size of `Drive/MT1 Heading Slip Deg` near and far from the tags. If normal noise is close to 3°, raise `HEADING_SLIP_WARNING_DEG`.

| Distance from tags | Typical slip deg | Max slip deg |
| --- | --- | --- |
| ~1.5 m | | |
| ~3 m | | |
| ~5 m | | |

- [ ] **Trigger it:** press Y while facing about 10° off, then drive to see 2+ tags. LEDs blink pink, `Vision/HeadingSlipWarning` = true, and LT aims visibly off.
- [ ] **Edge of the threshold:** press Y about 2° off (should stay quiet) and about 5° off (should trigger).
- [ ] **Priority:** with the warning on, hold LT and RT. Pink stays; green, orange and red never show.
- [ ] **Hold behavior:** with the warning on, turn away so no tags are visible. Pink stays on: losing the tags doesn't clear it.
- [ ] **Clear:** press Y facing correctly, look at the tags. Pink clears and the LEDs return to the normal state.
- [ ] **No false trips:** spin fast in place, and turn quickly while tags are in view. No pink.
- [ ] **Gyro drift over a match:** run a full-length practice (2:30) with lots of turning and watch whether slip deg creeps up over time.
- [ ] **Loop time:** watch the Driver Station for "Loop time of 0.02s overrun". If it shows up, see the loop-lag note in `SHOOTING-USING-POSE-PLAN.md` (Part B1).
- [ ] **Measure the MT1 check's cost:** even without overruns, check how much time the every-loop MegaTag1 check adds. In `Swerve.periodic()`, temporarily time the call:
  ```java
  double start = Timer.getTimestamp();
  updateHeadingSlip();
  m_driveTelemetry.log("Heading Slip Check ms", (Timer.getTimestamp() - start) * 1000.0);
  ```
  Drive around with tags in view for a minute and note the typical and max values. Under about 0.5 ms is negligible (the loop budget is 20 ms). If it's several ms, or the max spikes, use one of the plan's fixes: run it every few loops, or only while LT is held. Remove the timing log afterward.

| Condition | Typical ms | Max ms |
| --- | --- | --- |
| Tags in view (check runs) | | |
| No tags in view (check exits early) | | |

## 5. Before enabling interpolation: is the blending sensible?

Interpolation (commented out in `ShooterConstants`, `ShooterSubsystem` and `HashShoot`) draws a straight line between neighboring entries. Check the table supports that **before** uncommenting it.

**Spots in the current table where a straight-line blend may be wrong:**

| Between (m) | RPM | Angle | Boost | Concern |
| --- | --- | --- | --- | --- |
| 0.8 → 1.0 | 1900 → 2050 | 0.0 → 2.6 | 1.19 → 1.20 | Hood starts moving here; check 0.9 m |
| 1.0 → 1.2 | 2050 → 2040 | 2.6 → 5.85 | 1.20 → 1.21 | Big angle jump (+3.25) |
| 1.0 → 2.0 | 2050 → 1980 (dips) | rising | rising | RPM goes **down** as distance goes up, while angle rises. Real tuning or noise? |
| 1.6 → 1.8 | 2040 → 2000 | 8.0 → 10.0 | | RPM drops 40 |
| 2.0 → 2.2 | 1980 → 2000 | 12.61 → 13.4 | | Angle step shrinks from about 2.6 to 0.8 |
| 3.8 → 4.0 | 2160 → 2200 | 21.49 → 22.5 | 1.34 → **1.30** | Boost drops, then rises again to 1.32 |

- [ ] Plot RPM, angle and boost against distance (a spreadsheet is fine). Look for zig-zags. A zig-zag usually means one entry is off, not that the curve really bends.
- [ ] **Midpoint shots:** shoot 5 balls at the halfway points, by hand-setting the blended RPM/angle (e.g. via the tunables), **before** enabling interpolation.

| Midpoint (m) | Blended RPM | Blended angle | Blended boost | Made / 5 |
| --- | --- | --- | --- | --- |
| 0.9 | 1975 | 1.3 | 1.195 | |
| 1.1 | 2045 | 4.225 | 1.205 | |
| 1.9 | 1990 | 11.305 | 1.245 | |
| 2.3 | 2010 | 13.75 | 1.265 | |
| 3.1 | 2090 | 17.835 | 1.305 | |
| 3.9 | 2180 | 21.995 | 1.32 | |

- [ ] If the midpoints make about as often as the keys on either side, blending is sensible. Uncomment the interpolation code and delete the try/catch in `HashShoot` and the `rpmAngle` lookup in `getBoostMultiplier()`.
- [ ] If a midpoint misses, re-tune that region (add a key at the midpoint) before enabling.
- [ ] After enabling: drive slowly from 1 m to 4 m holding LT. RPM and hood move smoothly with no steps, and there's still only one boost per press.
- [ ] After enabling: beyond 4.4 m and under 0.0 m the values hold the end entry (no errors in the console).

## 6. Edge cases and exceptions

- [ ] **Alliance unknown:** before the Driver Station connects, `isRed` is "N/A" and the code aims at the **blue** hub. Confirm that once the DS connects on red, aiming switches to the red hub without a code restart.
- [ ] **Alliance changes after Y:** if the alliance is set after Y was pressed, the heading may be 180° off. Re-press Y and confirm the warning (section 4) catches it if not.
- [ ] **Beyond the table:** at about 5 m, `ForwardDistance` reads 4.4 (clamped), "in range" is false (> 4.0), and no rumble.
- [ ] **Very close:** under 0.4 m, the 0.0/0.4 entries are used. Check it doesn't shoot wildly.
- [ ] **Defense:** have another robot push yours, then check how far the pose drifts and how long MegaTag2 takes to correct it.
- [ ] **Auto:** run an auto that uses `HashShootAuto`. The logged distance at shot start matches where the robot actually is.
- [ ] **Auto aiming:** `BasicAuto`, `BasicAutoBlue`, `BasicAutoRed`, `BrightTryMakeAuto` and the PathPlanner named commands run `AlignAndMove` for 2 s before shooting, which now aims from the pose. Check it reaches aligned within the 2 s timeout, on both alliances. If it doesn't, the shot fires mis-aimed.
- [ ] **MegaTag1 seed at auto start:** after the seed, the pose heading and position are right, and there's no pink warning.
- [ ] **Limelight unplugged or rebooting:** aiming and shooting continue on odometry. The warning holds its last value (no new MegaTag1 frames).
- [ ] **Console:** no "Error occurred while looking up the HashShoot table" messages during any test.

## 7. Groundwork for auto-aiming while shuttling (passing)

Not implemented yet. The idea is for passes to aim at a point beside the hub. These tests show whether the pose and turning are good enough to try it.

- [ ] **Pose accuracy far from the hub:** drive to typical passing spots (neutral zone, far side). Note whether tags are visible there, and how far the pose drifts compared with tape measurements.
- [ ] **Time on odometry only:** at passing spots, how long has it been since the last MegaTag2 update? Passing aim needs the pose to stay good with no tags in view.
- [ ] **Where passes land today:** from 3–4 passing spots, record robot pose, heading, which pass (mid/far) and where the balls land. This gives the target point and the side to aim for.

| Robot pose (x, y, heading) | Pass (mid/far) | Landing spot (x, y) | Cleared the hub? |
| --- | --- | --- | --- |
| | | | |
| | | | |
| | | | |

- [ ] **Large turns:** hold LT from far away, facing away from the hub (about a 180° turn). Time how long until aligned, and check for overshoot. Passing aim would make turns this big.
- [ ] **Turning while driving fast:** hold LT at speed through the neutral zone. Does the robot hold heading on target while moving? Rough guide to whether shoot-on-the-move is realistic later.
- [ ] **Pass spread:** 5 passes from one spot. How wide do they land? If the spread is wider than the safe gap beside the hub, aim alone won't fix it.

## Later (not to do yet)

- [ ] Add `Vision/HeadingSlipWarning` to the Elastic layout (`deploy/Elastic-6220.json`). There's a TODO in `RobotContainer.publishDriverDashboardBooleans()`.
- [ ] Shuttling auto-aim to a point beside the hub (use section 7's results to pick the point and side).
- [ ] Interpolation (after section 5 passes).
- [ ] Shoot on the move: aim ahead by robot velocity × ball flight time.
- [ ] If loop overruns appear: run the heading-slip check less often (see plan Part B1).
- [ ] Clean up unused Limelight code: `Cameras.setPipelineForVisibleTag()`, `getFrontTagID()`, `isFrontPipelineNonZero()`, the `Shooter/LimelightTX` / `TargetVisible` logs, and the unreachable `-1.0` table entry.
- [ ] Optional: the camera is 0.014 m forward of robot center. Distance uses robot center, which is fine at the 0.2 m table step. Revisit only if interpolation is enabled and close shots are slightly off.
