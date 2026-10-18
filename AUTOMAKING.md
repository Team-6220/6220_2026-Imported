# Making PathPlanner Paths and Autos

How to make paths and autos in the PathPlanner app and get them into the auto chooser on Elastic.

All PathPlanner files live in `src/main/deploy/pathplanner/` (`paths/` and `autos/`). Open that folder as the project in the PathPlanner app. Changes reach the robot only when you **deploy code**.

## Field basics

- **Always draw for the blue alliance.** On red, `AutoBuilder` flips paths automatically by rotating them 180° around the field center. You never need a separate red version.
- Coordinates are **blue-origin**. +X points from the blue wall toward the red wall, and +Y points to the left as seen from the blue driver station.
- **0°** means facing the red wall (+X), and angles increase **counter-clockwise**. 90° faces +Y.
- Because of the flip, a correct auto on red ends 180° from the number you typed. That's what the field widget and the `where the bot think it is swerve degree` telemetry will show, since both use the blue-side origin. Seen from the red driver station, the robot faces the way you drew it.

## Heading vs. rotation

PathPlanner uses two separate angles, and mixing them up is the most common mistake.

| | **Heading** | **Rotation** |
| --- | --- | --- |
| What it is | The **direction of travel** along the path at that point | The way the **robot's front faces** |
| Set by | The waypoint's control points (dragging the handles) | Starting state, goal end state, and rotation targets |
| Affects | The path's shape | Which way the robot turns while driving |

A swerve robot can drive one way while facing another. For example, it can drive toward +X (heading 0°) while facing the left wall (rotation 90°). If the robot ends up facing the wrong way, check the **rotation** fields, not the path shape.

- **Ideal Starting State → Rotation:** which way the robot faces at the start. Place the robot facing this way.
- **Goal End State → Rotation:** which way the robot faces when the path ends.
- **Rotation Targets:** which way to face partway through the path.

## Rule for every auto: Reset Odometry on, `seedPose` first

Every `.auto` must:

1. Have **Reset Odometry** turned on (in the auto's settings; it's `"resetOdom": true` in the file).
2. Start with the **`seedPose`** named command as its very first block.

**Why:** Reset Odometry sets the robot's pose to the first path's starting pose. `seedPose` then replaces that with a vision pose (MegaTag1) **only if the front Limelight sees 2+ AprilTags**. Otherwise it keeps the path's starting pose.

At most starting spots the camera won't see 2 tags. So **place the robot exactly at the first path's start position and rotation**. Any placement error carries through the whole auto.

### Optional: `seedPose` at the end too

During the match, vision keeps correcting the robot's **position** but never its **heading**, which comes only from the gyro. So a few degrees of placement error at the start stays until someone presses Y, and it affects field-relative driving in teleop too.

If the auto ends somewhere the camera sees 2+ tags, add `seedPose` as the **last** block to re-set the heading from vision. If it doesn't see 2 tags, it does nothing, so it's safe to add.

- **Only when the robot is stopped.** It uses the newest camera frame as-is, so a moving robot gets a slightly out-of-date pose.
- **Closer tags are better.** Heading from two far-away tags can be off by a few degrees, which is worse than the gyro. Within about 3–4 m is best.
- **It overwrites the pose, it doesn't blend.** Before relying on it, check a few times that the vision heading agrees with what the robot actually faces.

## Making a path (single path, run with `FollowPathAuto`)

Use this for a one-path auto. For anything with more than one path, waits, or commands in sequence, make a `.auto` instead (next section).

1. In PathPlanner, create a path in `paths/`. Name it in camelCase with no spaces, for example `tinyPath`.
2. Set the waypoints, the **Ideal Starting State** rotation and the **Goal End State** rotation. Add rotation targets or event markers if needed.
3. Add it to the chooser in `RobotContainer.java`, next to the existing `tinyPath` line:

   ```java
   autoChooser.add("myPath", () -> new FollowPathAuto(s_Swerve, "myPath"));
   ```

   The string passed to `FollowPathAuto` must match the `.path` file name, without `.path`.
4. Deploy.

`FollowPathAuto` resets the pose to the path's start, runs `seedPose`, then follows the path. So the rule above is handled for you.

## Making an auto (`.auto`, loaded automatically)

1. In PathPlanner, create an auto in `autos/`.
2. Turn on **Reset Odometry**.
3. Add a **Named Command** block for `seedPose` as the first block.
4. Add path blocks, named commands, waits, and groups (sequential, parallel, deadline, race).
5. Deploy. **No code changes needed:** every `.auto` shows up in the chooser as **`PP: <autoName>`**.

## Named commands you can use

These are registered in `RobotContainer.registerNamedCommands()`. Names are **case-sensitive** and must match exactly.

| Name | What it does | Ends on its own? |
| --- | --- | --- |
| `seedPose` | Sets the pose from vision if 2+ tags are visible; otherwise does nothing | Yes, instantly |
| `intake` | Runs the intake rollers inward and moves the arm down | **No** |
| `outtake` | Runs the intake rollers outward | **No** |
| `armDown` | Moves the arm to the down position | Yes, when the arm reaches it |
| `armUp` | Moves the arm to the up position | Yes, when the arm reaches it |
| `shoot` | Sets the shooter angle and RPM from the Limelight distance, and runs the belt | **No** |
| `align` | Turns the robot to aim at the visible tag (drives the robot) | Yes, after 2 s |
| `alignAndShoot` | `align` (2 s), then `shoot` (4 s) | Yes, after about 6 s |

### Using commands that never end

`intake`, `outtake` and `shoot` keep running until something stops them. In a sequential list, the auto gets stuck on them. Use one of these instead:

- **Deadline group:** put a path or a wait as the deadline, with the command beside it. The command stops when the deadline finishes. For example, deadline = the path to the game piece, plus `intake`.
- **Race group:** whichever block finishes first stops the others.
- **Event marker zone on a path:** the command runs only while the robot is in that part of the path.

### Commands that drive the robot

`align` and `alignAndShoot` take control of the drivetrain. Use them **only between paths**, never as an event marker during a path, because they'd fight the path follower for the drive.

### How `shoot` picks its distance

`shoot` and `alignAndShoot` measure the Limelight distance at the instant they start, then use that RPM and angle for the whole shot. If a tag is visible at that moment, the shot works normally. But:

- If that one frame has no tag (the robot is still settling, or the image is blurry), the whole shot uses a 100 RPM test value, even if the tag appears a moment later.
- If the robot moves during the shot, RPM and angle don't update.

So finish the path **stopped and facing the hub tags** before `shoot` starts. A short wait block before it helps.

## Checklist before running a new auto

- [ ] Drawn for blue
- [ ] Reset Odometry on, `seedPose` first (`.auto` only)
- [ ] Commands that never end are inside a deadline/race group or a marker zone
- [ ] No `align` / `alignAndShoot` as event markers
- [ ] Robot placed at the first path's start position **and** rotation
- [ ] Code deployed after saving in PathPlanner
