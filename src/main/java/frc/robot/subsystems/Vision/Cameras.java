// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import com.limelightvision.Limelight;
import com.limelightvision.LimelightResults;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateConfig;
import com.limelightvision.PoseEstimateType;
import com.limelightvision.Status;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;

/** Shared LimelightLib 2 camera instances. */
public final class Cameras {
  private Cameras() {}

  private static final double CAMERA_FORWARD_METERS = 0.0142638526;
  private static final double CAMERA_LEFT_METERS = 0.0;
  private static final double CAMERA_UP_METERS = 0.5161723992;
  private static final double CAMERA_ROLL_DEGREES = 0.0;
  private static final double CAMERA_PITCH_DEGREES = -55;
  private static final double CAMERA_YAW_DEGREES = 0.0;
  

  // MegaTag1 filtering, only used to seed the starting pose (including heading) when auto starts.
  // Requires 2+ tags since single-tag MT1 can flip to an ambiguous pose.
  private static final PoseEstimateConfig MT1_CONFIG =
      PoseEstimateConfig.defaultMT1()
          .withMinTagCount(2)
          .withMaxAvgTagDistance(6.0)
          .withMinAvgTagArea(0.05) // 0-100, percentage of image area
          .withFieldBounds(16.541, 8.069) // 2026 welded field
          .withFieldBoundsMargin(0.5);

  // MegaTag2 filtering and trust scaling. Starting values from the LimelightLib 2 docs; tune on
  // the robot.
  private static final PoseEstimateConfig MT2_CONFIG =
      PoseEstimateConfig.defaultMT2()
          .withMinTagCount(1)
          .withMaxSingleTagAmbiguity(1.0) // MT2 handles ambiguous perspectives
          .withMaxSingleTagDistance(0.0) // 0 disables this check
          .withMaxAvgTagDistance(8.0)
          .withMinAvgTagArea(0.02) // 0-100, percentage of image area
          .withFieldBounds(16.541, 8.069) // 2026 welded field
          .withFieldBoundsMargin(0.5)
          .withStdDevXY(0.3, 0.0001, 2.0)
          .withStdDevTheta(
              PoseEstimateConfig.UNTRUSTED,
              PoseEstimateConfig.UNTRUSTED,
              PoseEstimateConfig.UNTRUSTED) // never fuse vision heading
          .withStdDevDistanceScaling(0.5, 0.0, 8.0) // scale by sqrt(distance)
          .withStdDevTagCountDivision(0.5); // trust grows with sqrt(tag count)

  /** Front-facing Limelight used for aiming and MegaTag2 localization. */
  public static final Limelight FRONT =
      new Limelight(
              "limelight-front",
              CAMERA_FORWARD_METERS,
              CAMERA_LEFT_METERS,
              CAMERA_UP_METERS,
              CAMERA_ROLL_DEGREES,
              CAMERA_PITCH_DEGREES,
              CAMERA_YAW_DEGREES)
          .withPoseEstimateConfig_MT1(MT1_CONFIG)
          .withPoseEstimateConfig_MT2(MT2_CONFIG);

  static {
    FRONT.setPipelineIndex(0);
  }

  /**
   * Gets a full field pose (including heading) from MegaTag1, which doesn't use the gyro. Uses the
   * newest frame without consuming the queue, since Swerve.periodic() reads the queue for MT2.
   *
   * @return The blue-origin pose if the camera is live and sees 2+ tags, otherwise empty
   */
  public static Optional<Pose2d> getMT1StartPose() {
    if (FRONT.getStatus() != Status.OK) return Optional.empty();
    PoseEstimate estimate = FRONT.getPoseEstimate(PoseEstimateType.MT1_WPIBLUE);
    if (!estimate.isValid() || estimate.fieldedTagCount < 2) return Optional.empty();
    return Optional.of(estimate.pose);
  }

  /**
   * Gets the ID of the primary visible AprilTag on the front camera.
   *
   * @return The tag ID, or -1 if no tag is visible
   */
  public static int getFrontTagID() {
    if (!FRONT.hasTarget()) return -1;
    LimelightResults results = FRONT.getLatestResults();
    if (results.fiducialTargets == null || results.fiducialTargets.length == 0) return -1;
    return results.fiducialTargets[0].fiducialId;
  }

  /**
   * Whether the front camera is on any pipeline other than the default (0), i.e. a tag-specific
   * pipeline selected by {@link #setPipelineForVisibleTag()}.
   *
   * @return true if the front camera's current pipeline index is not 0
   */
  public static boolean isFrontPipelineNonZero() {
    return FRONT.getCurrentPipelineIndex() != 0;
  }

  /** Switches the front camera to the pipeline for the currently visible tag. */
  public static void setPipelineForVisibleTag() {
    int tag = getFrontTagID();
    if (tag == 2 || tag == 18) {
      FRONT.setPipelineIndex(1);
    } else if (tag == 11 || tag == 27) {
      FRONT.setPipelineIndex(2);
    } else if (tag == 10 || tag == 26) {
      FRONT.setPipelineIndex(3);
    } else if (tag == 9 || tag == 25) {
      FRONT.setPipelineIndex(4);
    } else if (tag == 8 || tag == 24) {
      FRONT.setPipelineIndex(5);
    } else if (tag == 5 || tag == 21) {
      FRONT.setPipelineIndex(6);
    } else {
      FRONT.setPipelineIndex(0);
    }
  }
}
