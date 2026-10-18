package frc.robot.commands;

import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Vision.Cameras;
import java.util.Optional;
import org.wpilib.command2.InstantCommand;
import org.wpilib.math.geometry.Pose2d;

/**
 * Resets the pose (including heading) from MegaTag1 if the front Limelight sees 2+ AprilTags, so
 * the robot doesn't need to be placed exactly. Does nothing otherwise, keeping whatever pose was
 * set before it.
 *
 * <p>Put this first in an auto, after the start pose reset. In a PathPlanner .auto, keep "Reset
 * Odometry" on and add the "seedPose" named command as the first block: PathPlanner resets to the
 * path start before any commands run, so this overrides it only when vision is good.
 */
public class SeedPoseFromVision extends InstantCommand {
  public SeedPoseFromVision(Swerve swerve) {
    super(
        () -> {
          Optional<Pose2d> visionPose = Cameras.getMT1StartPose();
          if (visionPose.isPresent()) {
            swerve.setPose(visionPose.get());
            System.out.println("Starting pose from MegaTag1: " + visionPose.get());
          } else {
            System.out.println("MegaTag1 didn't see 2+ tags, keeping starting pose: "
                + swerve.getPose());
          }
        });
  }

  // Safe to run while disabled, e.g. from a dashboard button before the match
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
