package frc.robot.commands.Autos.SamAuto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import frc.robot.AutoConstants;
import frc.robot.commands.AlignAndMove;
import frc.robot.commands.ArmToPositionCommand;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.HashShootAuto;
import frc.robot.commands.TestRollerCommand;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Intake.ArmSubsystem;
import frc.robot.subsystems.Intake.BeltSubsystem;
import frc.robot.subsystems.Shooter.AnglerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.Constants;

import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import static org.wpilib.units.Units.MetersPerSecond;

public class BrightTryMakeAuto extends SequentialCommandGroup{
      /** Starting pose (toptostation.path first anchor, idealStartingState rotation). */
  private static final Pose2d START_POSE = new Pose2d(3.43, 4.05, Rotation2d.fromDegrees(0));
  private final ArmSubsystem arm;
  private final CommandXboxController controller;
  /** Poses to drive to, in order (last one = toptostation.path end anchor + goalEndState). */
  private static Pose2d[] ALIGNTOOUTPOST = {
        new Pose2d(1.7395, 5.946, Rotation2d.fromDegrees(180)),

    new Pose2d(1.176, 5.946, Rotation2d.fromDegrees(180)),
  };
  private static Pose2d[] INTAKEWAYPOINTS = {
        new Pose2d(0.7, 5.946, Rotation2d.fromDegrees(180)),
  };
    private static Pose2d[] SHOOTWAYPOINTS = {
        new Pose2d(2.1, 5.12, Rotation2d.fromDegrees(0)),
        new Pose2d(2.15, 3.98, Rotation2d.fromDegrees(0)),
  };

  // Red alliance versions of the poses above. The 2026 field is rotationally symmetric, so each
  // pose is rotated 180 degrees about field center: x -> 16.541 - x, y -> 8.069 - y, heading + 180.
  private static final Pose2d RED_START_POSE = new Pose2d(13.111, 4.019, Rotation2d.fromDegrees(180));
  private static Pose2d[] RED_ALIGNTOOUTPOST = {
        new Pose2d(14.8015, 2.123, Rotation2d.fromDegrees(0)),

    new Pose2d(15.365, 2.123, Rotation2d.fromDegrees(0)),
  };
  private static Pose2d[] RED_INTAKEWAYPOINTS = {
        new Pose2d(15.841, 2.123, Rotation2d.fromDegrees(0)),
  };
    private static Pose2d[] RED_SHOOTWAYPOINTS = {
        new Pose2d(14.441, 2.949, Rotation2d.fromDegrees(180)),
        new Pose2d(14.391, 4.089, Rotation2d.fromDegrees(180)),
  };

  /** Give up on any single waypoint after this long so auto can't hang. */
  private static final double WAYPOINT_TIMEOUT_SECONDS = 4.0;

  /** Creates a new SamAutoV2. */
  public BrightTryMakeAuto(Swerve swerve, ArmSubsystem arm,CommandXboxController controller, AnglerSubsystem angler, ShooterSubsystem shooter, BeltSubsystem belt) {
    this.arm = arm;
    this.controller = controller;
    addRequirements(swerve);
    // Alliance is read when this auto is built, so build it after the alliance is known.
    boolean isRed = Constants.isRed.equals("red");
    Pose2d startPose = isRed ? RED_START_POSE : START_POSE;
    Pose2d[] alignToOutpost = isRed ? RED_ALIGNTOOUTPOST : ALIGNTOOUTPOST;
    Pose2d[] intakeWaypoints = isRed ? RED_INTAKEWAYPOINTS : INTAKEWAYPOINTS;
    Pose2d[] shootWaypoints = isRed ? RED_SHOOTWAYPOINTS : SHOOTWAYPOINTS;
    addCommands(new InstantCommand(() -> swerve.zeroHeading(), swerve));
    addCommands(new InstantCommand(() -> swerve.setPose(startPose)));
    for (Pose2d waypoint : alignToOutpost) {
      addCommands(new DriveToPose(swerve, waypoint).withTimeout(WAYPOINT_TIMEOUT_SECONDS));
    };
    addCommands(new InstantCommand(()-> AutoConstants.translationMaxVelMpsTN.set(MetersPerSecond.of(0.5).in(MetersPerSecond))));
    addCommands(new TestRollerCommand(true)
      .alongWith(new ArmToPositionCommand(this.arm, -2)).withDeadline(
        new DriveToPose(swerve, intakeWaypoints[0]).withTimeout(WAYPOINT_TIMEOUT_SECONDS).andThen(new WaitCommand(0.5))
      )
      );
          addCommands(new InstantCommand(()-> AutoConstants.translationMaxVelMpsTN.set(MetersPerSecond.of(5).in(MetersPerSecond))));

  for (Pose2d waypoint : shootWaypoints) {
      addCommands(new DriveToPose(swerve, waypoint).withTimeout(WAYPOINT_TIMEOUT_SECONDS));
    };
    addCommands(new AlignAndMove(swerve, this.controller, this.controller.rightBumper()).withTimeout(2),
        new HashShootAuto(angler, shooter, belt).withTimeout(4));


    addCommands(new InstantCommand(() -> System.out.println("Done with SamAutoV2!")));
  }

}