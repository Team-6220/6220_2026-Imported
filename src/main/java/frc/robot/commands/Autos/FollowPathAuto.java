package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import frc.robot.commands.SeedPoseFromVision;
import frc.robot.subsystems.Drive.Swerve;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.math.geometry.Pose2d;

/**
 * Follows a single path from deploy/pathplanner/paths/ with PathPlanner. Event markers in the path
 * run their named commands (see RobotContainer.registerNamedCommands). For multiple paths, waits,
 * and commands in a row, make a .auto in PathPlanner instead; those show up in the chooser
 * automatically.
 *
 * <p>When auto starts, the pose is reset to the path's first point, then {@link SeedPoseFromVision}
 * overrides it from MegaTag1 if the front Limelight sees 2+ AprilTags.
 *
 * <p>Requires Swerve.configureAutoBuilder() to have been called. The path is flipped for red
 * alliance by AutoBuilder, so build this when auto starts (the chooser does) so the alliance is
 * known.
 */
public class FollowPathAuto extends SequentialCommandGroup {
  /**
   * Creates a new FollowPathAuto.
   *
   * @param pathName path file name without ".path", e.g. "stationtoshoot"
   */
  public FollowPathAuto(Swerve swerve, String pathName) {
    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile(pathName);
    } catch (IOException | ParseException | FileVersionException e) {
      throw new RuntimeException("Failed to load " + pathName + ".path", e);
    }

    // Fallback start: the path's first point (flipped to match AutoBuilder on red)
    PathPlannerPath startPath = AutoBuilder.shouldFlip() ? path.flipPath() : path;
    Pose2d startPose = startPath.getStartingHolonomicPose().orElseThrow();

    addCommands(
        new InstantCommand(() -> swerve.setPose(startPose)),
        new SeedPoseFromVision(swerve),
        AutoBuilder.followPath(path),
        new InstantCommand(() -> System.out.println("Done with " + pathName + "!")));
  }
}
