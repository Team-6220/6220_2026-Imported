package frc.robot.subsystems.Shooter;

import frc.robot.Constants;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Hub distance and aim heading from the robot's field pose (odometry + MegaTag2).
 *
 * <p>All poses are blue-origin, so each hub has one fixed field coordinate and red uses the red
 * hub directly (no flipping). The aim heading and the pose heading are in the same field frame, so
 * the heading error works the same on both alliances.
 */
public final class HubTargeting {
  private HubTargeting() {}

  // Hub centers, blue-origin field coordinates (m). Average of each hub's 8 AprilTags
  // in WPILib's 2026-rebuilt-welded.json.
  public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6256, 4.0346);
  public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9154, 4.0346);

  /**
   * Gets our alliance's hub center. Falls back to blue when the alliance is unknown ("N/A"), the
   * same as {@code Swerve.zeroHeading()}.
   */
  public static Translation2d getHubCenter() {
    return Constants.isRed.equals("red") ? RED_HUB_CENTER : BLUE_HUB_CENTER;
  }

  /**
   * Floor-plane (XY) distance from the robot's center to the hub's center, the same quantity the
   * shot table was recorded against. The camera is only 0.014 m forward of center, well under the
   * table's 0.2 m step.
   */
  public static double getDistanceMeters(Pose2d robotPose) {
    return robotPose.getTranslation().getDistance(getHubCenter());
  }

  /**
   * Shuttle target: on the trench/bump boundary line on the robot's side of the field (by y),
   * inside our alliance zone. Passes aimed here land on our side without hitting the hub.
   */
  public static Translation2d getShuttleTarget(Pose2d robotPose) {
    boolean red = Constants.isRed.equals("red");
    double x =
        red
            ? ShooterConstants.FIELD_LENGTH_M - ShooterConstants.SHUTTLE_TARGET_X_FROM_WALL_M
            : ShooterConstants.SHUTTLE_TARGET_X_FROM_WALL_M;
    double centerY = BLUE_HUB_CENTER.getY();
    double y =
        robotPose.getY() >= centerY
            ? centerY + ShooterConstants.SHUTTLE_TARGET_Y_OFFSET_M
            : centerY - ShooterConstants.SHUTTLE_TARGET_Y_OFFSET_M;
    return new Translation2d(x, y);
  }

  /** Floor-plane (XY) distance from the robot's center to the shuttle target. */
  public static double getShuttleDistanceMeters(Pose2d robotPose) {
    return robotPose.getTranslation().getDistance(getShuttleTarget(robotPose));
  }

  /**
   * Field heading that points the robot's front (shooter side) at the hub center. Keeps the current
   * heading if the robot is exactly on the hub center, where no direction exists.
   */
  public static Rotation2d getAimHeading(Pose2d robotPose) {
    return getAimHeading(robotPose, getHubCenter());
  }

  /** Field heading that points the robot's front at the given field point. */
  public static Rotation2d getAimHeading(Pose2d robotPose, Translation2d target) {
    return target.minus(robotPose.getTranslation()).getAngle().orElse(robotPose.getRotation());
  }

  /** Aim heading minus the robot's heading, wrapped to -180 to 180 degrees. */
  public static double getHeadingErrorDegrees(Pose2d robotPose) {
    return getHeadingErrorDegrees(robotPose, getHubCenter());
  }

  /** Heading error to the given field point, wrapped to -180 to 180 degrees. */
  public static double getHeadingErrorDegrees(Pose2d robotPose, Translation2d target) {
    return getAimHeading(robotPose, target).minus(robotPose.getRotation()).getDegrees();
  }

  /** Whether the robot is facing the hub center within the given tolerance. */
  public static boolean isAligned(Pose2d robotPose, double toleranceDeg) {
    return isAligned(robotPose, getHubCenter(), toleranceDeg);
  }

  /** Whether the robot is facing the given field point within the given tolerance. */
  public static boolean isAligned(Pose2d robotPose, Translation2d target, double toleranceDeg) {
    return Math.abs(getHeadingErrorDegrees(robotPose, target)) <= toleranceDeg;
  }
}
