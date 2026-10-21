# Pose-based hub aiming and shot distance

## Context
LT (`AlignAndFlywheels` → `AlignAndMove` + `HashShoot`) currently aims with Limelight `tx` and reads distance from `fiducialTargets[0]` camera-space X. The `rpmAngle` table was recorded with the 2026 code, where the Limelight 3D offset made that number the **floor-plane (XY) distance from the camera to the hub center**. The port to LimelightLib 2 and the firmware wipe lost that offset, so aim and distance now point at the tag face.

We replace the Limelight path with the Swerve pose (`s_Swerve.getPose()`, already fused from odometry + MegaTag2). The pose gives XY distance to the hub center for shooting and the heading to the hub center for aiming. The distance means the same thing as in 2026, so **the table is unchanged**. The existing commands and structure stay; only their data source changes.

## Changes

### 1. New `frc/robot/subsystems/Shooter/HubTargeting.java` (static helpers, next to `ShooterConstants`)
Package `frc.robot.subsystems.Shooter`. `ShooterSubsystem` uses it from the same package. `AlignAndMove` and `RobotContainer` import `frc.robot.subsystems.Shooter.HubTargeting`, the same way they already import the other Shooter classes.

**Getting the distance to the hub center takes three steps:**

1. **Check the alliance.** Read `Constants.isRed`, which `Robot.java` already keeps updated from the Driver Station (`"red"`, `"blue"` or `"N/A"`).
2. **Pick that alliance's hub center.** All poses are blue-origin (the Swerve pose and MegaTag2 `MT2_WPIBLUE`), so each hub has one fixed field coordinate, and red uses the red hub's coordinate rather than a flipped blue one:
   ```java
   // Hub centers, blue-origin field coordinates (m). Average of each hub's 8 AprilTags
   // in WPILib's 2026-rebuilt-welded.json.
   public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6256, 4.0346);
   public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9154, 4.0346);

   public static Translation2d getHubCenter() {
     return Constants.isRed.equals("red") ? RED_HUB_CENTER : BLUE_HUB_CENTER;
   }
   ```
   - Blue hub tags: 18–21, 24–27. Red hub tags: 2–5, 8–11. Each hub's tags sit symmetrically around its center (for example, tags 26 and 10 face out at x = 4.0219 and 12.5192, and tags 21/18 sit at y = 4.6380/3.4312).
   - `"N/A"` (no Driver Station yet) falls back to blue, the same as `Swerve.zeroHeading()`.
   - If you have official hub-center coordinates, they replace these two constants and nothing else changes.
3. **Measure robot center to hub center in the floor plane:**
   ```java
   public static double getDistanceMeters(Pose2d robotPose) {
     return robotPose.getTranslation().getDistance(getHubCenter());
   }
   ```
   - `Pose2d` is 2D, so this is the XY distance `sqrt(dx² + dy²)`, the same quantity the table was recorded against. Height plays no part.
   - The 2026 measurement came from the camera, which is 0.014 m forward of robot center (`Cameras.CAMERA_FORWARD_METERS`). When aimed at the hub, that is 1.4 cm, far below the table's 0.2 m step, so robot center is used.

**Worked example:** a blue robot at (2.0, 4.0) gives dx = 2.6256 and dy = 0.0346, so the distance is 2.626 m, which becomes table key 2.6. The same robot spot on red, mirrored to (14.541, 4.0), gives the same 2.626 m to the red hub.
- `getAimHeading(Pose2d pose)`: `getHubCenter().minus(pose.getTranslation()).getAngle()`. The shooter faces the same way as the front camera that `tx` used.
- `getHeadingErrorDegrees(Pose2d pose)`: `getAimHeading(pose).minus(pose.getRotation()).getDegrees()`. `Rotation2d` wraps the angle.
- `isAligned(Pose2d pose, double toleranceDeg)`: `Math.abs(getHeadingErrorDegrees(pose)) <= toleranceDeg`.

**Alliance check for the aim angle (verified, no special red handling needed):**
- The pose heading and the aim heading are both in the same blue-origin field frame. On red, the pose heading is already 180° off blue because `zeroHeading()` resets it to π. The aim heading comes from the red hub's field coordinate, so it is 180°-ish too. Their difference is alliance-independent.
- **Blue example:** robot at (2.0, 4.0), facing the hub (+x). Aim = atan2(0.0346, 2.6256) = **+0.75°**. Pose heading after Y facing away from the blue DS = 0°. Error = 0.75°, so it is aligned.
- **Red example:** mirrored robot at (14.541, 4.0), facing the red hub (−x). Aim = atan2(0.0346, −2.6256) = **+179.25°**. Pose heading after Y facing away from the red DS = 180°. Error = −0.75°, so it is aligned and the turn is small.
- **Wrap at ±180°:** red aim of +179° with the robot at −179° gives an error of +2° through `Rotation2d.minus`, not 358°. The turn PID already has `enableContinuousInput(-π, π)` (Swerve.java:172), so the goal never sends the robot the long way around.
- **Do not** use `AlienceColorCoordinateFlip` or the `IOConstants` red stick inversion here. Those are for driver-relative inputs and authored blue paths. Flipping the aim would double-flip it.
- Verification step 2 runs the sim on both alliances to confirm.

### 2. `subsystems/Shooter/ShooterSubsystem.java`
- The constructor takes `Supplier<Pose2d> poseSupplier` and stores it.
- `getDist()` keeps its name and rounding, but its input becomes `HubTargeting.getDistanceMeters(poseSupplier.get())`. The existing round-to-0.2 m math stays.
- The result is clamped to the table's key range (0.0–4.4) so `rpmAngle.get()` never returns null. The `hasTarget()` and `-1` early returns go away.
- `HashShoot`, `HashShootAuto`, `getBoostMultiplier()`, the `inRange` trigger and `Shooter/ForwardDistance` all switch with no edits of their own.
- Remove the now-unused `LimelightResults` / `Cameras` imports if nothing else uses them. The `Shooter/LimelightTX` and `TargetVisible` logs can stay.

### 3. `commands/AlignAndMove.java` (same command, same constructor)
- `initialize`:
  - Keep `resetModulesToAbsolute()` and `resetTurnController()`.
  - Goal is `Radians.of(HubTargeting.getAimHeading(s_Swerve.getPose()).getRadians())`, using the same `setTurnControllerGoal` that `DriveToPose` uses.
  - Remove `setPipelineForVisibleTag()`.
- `execute`:
  - Remove the `ticks` and pipeline logic.
  - Update the goal from the pose each loop.
  - `aligned = HubTargeting.isAligned(pose, 2)`.
  - Aligned and sticks idle (≤ 0.1): `s_Swerve.lockWheels()` **only**.
  - Otherwise: `drive(translation, aligned ? 0.0 : s_Swerve.getTurnPidSpeed(), !robotCentric, false)`.
  - This removes today's drive-then-lock fight in one loop.
- `end`: `stopDriving()`. Remove `Cameras.FRONT.setPipelineIndex(0)` and the debug print.

### 4. `commands/HashShoot.java`
Move the `rpmAngle` lookup (with its try/catch) from `initialize()` into `execute()`, so RPM and angle follow the pose while the driver moves. `resetShootingState()` stays in `initialize()`. `HashShootAuto` keeps reading once at start, since autos shoot standing still.

> **Watch for jitter.** Looking up the table every loop means the RPM and angle setpoints jump whenever the distance crosses a 0.2 m key. That can come from the robot moving or from pose noise near a boundary, for example MegaTag2 corrections nudging the pose back and forth. The flywheel and angler then chase changing setpoints mid-shot. We keep this change for now. **If shooting is jittery, undo this first:** move the lookup back into `initialize()` so each LT press locks one distance. Interpolation or a small hysteresis on the key are later alternatives.

### 5. `RobotContainer.java` (wiring only)
- `new ShooterSubsystem()` becomes `new ShooterSubsystem(s_Swerve::getPose)`. `s_Swerve` is declared on the line above.
- The `aligned` trigger and `publishDriverDashboardBooleans()` use `HubTargeting.isAligned(s_Swerve.getPose(), ALIGN_TOLERANCE_DEG)` instead of `hasTarget() && |tx| < tol`. LEDs and rumble then match what LT aims at.

### Left as is
- `AlignAndFlywheels`, `Cameras`, the shot table, and the autos. `Swerve` only changes for Part B below; its MegaTag2 fusion in `periodic()` stays as is.
- `Cameras.setPipelineForVisibleTag` and `isFrontPipelineNonZero` become unused by LT and can be deleted later.

## Verification
1. `./gradlew build` with the WPILib JDK.
2. Sim: place the robot at known poses on Field2d, check `Shooter/ForwardDistance` against hand math, and hold LT to confirm it turns to the hub on both alliances.
3. Robot, distance: tape-measure robot-center-to-hub-center floor distance at about 1.5, 2.5 and 3.5 m. The logged value should be within one 0.2 m step.
4. Robot, aim: hold LT from several angles. It turns to hub center, the LEDs and rumble fire when within 2°, and the wheels lock only with idle sticks.
5. Robot, shots: shoot from the same spots to confirm the table carries over.
6. Camera blocked: cover the Limelight, drive a short loop, hold LT. It still aims and shoots, on odometry alone.

## Risks
- **Pose quality drives everything.** Press Y facing the right way, since MegaTag2 needs a correct heading. The MegaTag1 seed at auto start helps. Wheel slip under defense drifts the pose until MegaTag2 corrects it.
- **Jittery shots:** the first suspect is the per-loop `rpmAngle` lookup in `HashShoot` (see section 4). Revert it to `initialize()` first.
- **Alliance unknown:** while `Constants.isRed` is `"N/A"`, the code aims at the blue hub. That only matters before the Driver Station connects.

## Later (not actionable yet)
- **Shuttling should aim to the side of the hub.** Passing (`PassToAlliance`, on RB for mid and B for far) currently doesn't turn the robot at all: it only requires the angler, shooter and belt, and fires at fixed RPM/angle. Once pose aiming is proven, shuttling should always aim at a point beside the hub so passes clear it.
  - It could reuse `HubTargeting`: a `getPassTarget()` point offset sideways from `getHubCenter()`, for example toward the side the robot is on.
  - It would turn with the same turn-PID approach as `AlignAndMove`.
  - The offset point and the choice of side are still to be decided. Don't implement this as part of this plan.

---

# Part B: heading-slip warning (MegaTag1 vs pose heading)

## Why
Aim and distance now trust the pose. MegaTag2 never corrects heading: `MT2_CONFIG` sets theta std devs to `UNTRUSTED`, and MT2 itself uses our heading as an input. If the gyro drifts or Y was pressed facing the wrong way, everything aims off with no visible sign. MegaTag1 solves heading from 2+ tags with no gyro, so it is an independent check.

## Changes

### B1. `subsystems/Drive/Swerve.java`
- **Constant**, next to `MAX_MT2_YAW_RATE_DEG_PER_SEC` (Swerve.java:97):
  ```java
  /** Pose heading vs MegaTag1 heading difference that raises the heading-slip warning. */
  public static final double HEADING_SLIP_WARNING_DEG = 3.0;
  ```
- **Fields:** `private boolean headingSlipping = false;` and `private double lastMt1Timestamp = -1;` (to skip repeated frames).
- **Check, in `periodic()`** right after the MT2 loop (inside the "vision stuff" block):
  - `PoseEstimate mt1 = Cameras.FRONT.getPoseEstimate(PoseEstimateType.MT1_WPIBLUE)`. This is the same call `Cameras.getMT1StartPose()` makes. It peeks the newest frame without touching the MT2 queue. It needs a timestamp, so call it directly; `getMT1StartPose()` returns no timestamp.
  - Use it only if `Cameras.FRONT.getStatus() == Status.OK`, `mt1.isValid()`, `mt1.fieldedTagCount >= 2` (single-tag MT1 can flip), `mt1.timestampSeconds != lastMt1Timestamp`, and `!spinningTooFast`.
  - Compare against the pose **at the frame's timestamp**: `poseEstimator.sampleAt(mt1.timestampSeconds).orElse(getPose())`. `sampleAt` exists in 2027 alpha 7 `PoseEstimator`. This avoids false trips while turning, when camera latency would otherwise show up as heading error.
  - `slipDeg = mt1.pose.getRotation().minus(sampledPose.getRotation()).getDegrees()`. This wraps, so blue and red behave the same.
  - `headingSlipping = Math.abs(slipDeg) > HEADING_SLIP_WARNING_DEG`.
  - With no valid MT1 frame, `headingSlipping` **holds its last value**. Losing sight of the tags doesn't clear a real slip; a valid agreeing frame does, for example after re-zeroing with Y.
> **Possible loop lag (no action now).** This check runs every 20 ms loop: an MT1 read, a `sampleAt` lookup and the math.
> - Each piece should be cheap. The MT1 read peeks the frame LimelightLib already decoded, and `sampleAt` is a buffer lookup. But it adds to `Swerve.periodic()`, which already does the MT2 work.
> - If loop overruns ("Loop time of 0.02s overrun") or sluggish driving show up, these are options for later:
>   - Run the check every N loops (e.g. 5 Hz) instead of every loop.
>   - Only run it while LT is held or when the robot isn't moving.
>   - Skip it when `Cameras.FRONT.hasTarget()` is false, before any other work.
> - Measure first: compare loop time in the logs with and without the check.

- **Getter:** `public boolean isHeadingSlipping()`.
- **Telemetry, in `updateTelemetry()`:** log `"MT1 Heading Slip Deg"` (the last `slipDeg`) for tuning the 3° threshold, and `"Heading Slip Warning"`.

### B2. `RobotContainer.java`
- **Trigger:** `Trigger headingSlip = new Trigger(s_Swerve::isHeadingSlipping).debounce(0.25);`. The debounce keeps one noisy MT1 frame at long range from flashing the LEDs.
- **Highest-priority LED binding:**
  ```java
  // Heading slip (MegaTag1 disagrees with pose heading) -> Flashing Pink. Highest priority.
  headingSlip.whileTrue(s_LED.runPattern(s_LED.blink(s_LED.solidPink(), 0.15)));
  ```
- **Mutual exclusion:** add `.and(headingSlip.negate())` to all four existing LED bindings (RobotContainer.java:274–289): aligned/not ready (flashing green), ready-or-shooting/not aligned (flashing orange), ready (solid green), shooting+aligned (flashing red).
  - Then only one LED command can ever be scheduled, and pink wins.
  - When the slip clears, the normal state's trigger turns true again and takes over.
  - The `AdressableLEDs` default (fire gradient) runs only when none is active, as today.
- **Rumble** stays unchanged, since it isn't an LED state.
- **Telemetry**, in `publishDriverDashboardBooleans()` next to `Shooter/Aligned`:
  ```java
  m_visionTelemetry.log("HeadingSlipWarning", s_Swerve.isHeadingSlipping());
  // TODO: add Vision/HeadingSlipWarning to the Elastic layout (deploy/Elastic-6220.json) later.
  ```
- Elastic-6220.json is **not** edited.

## Verification (Part B)
1. Sim or robot with tags in view: no warning in normal driving; `MT1 Heading Slip Deg` stays small (note the typical noise to judge the 3°).
2. Press Y while facing about 10° off, or nudge the heading in sim. In view of 2+ tags, the LEDs blink pink, `Vision/HeadingSlipWarning` = true, and pink overrides green/orange/red even while holding LT or RT.
3. Press Y facing correctly, in view of tags. The warning clears and the LEDs return to the normal state.
4. Spin fast in place. No false warning, since the spin gate and `sampleAt` cover this.
5. Watch the Driver Station for loop overrun messages. If they appear, see the loop-lag note in B1.
