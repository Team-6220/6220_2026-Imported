package frc.robot.commands;

import static org.wpilib.units.Units.Radians;

import frc.robot.IOConstants;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Shooter.HubTargeting;
import java.util.function.BooleanSupplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Turns the robot to face our hub's center using the swerve pose (odometry + MegaTag2) while the
 * driver keeps control of translation. Works with no tag in view.
 */
public class AlignAndMove extends Command {
  /** Heading error to the hub center that counts as aligned. Also used by RobotContainer. */
  public static final double ALIGN_TOLERANCE_DEG = 2.0;
  private static final double STICK_IDLE_THRESHOLD = 0.1;

  private Swerve s_Swerve;
  private BooleanSupplier robotCentricSup;
  private CommandXboxController driver;

  public AlignAndMove(
      Swerve s_Swerve, CommandXboxController driver, BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    this.driver = driver;
    this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void initialize() {
    // Initialize so that the swerve doesn't become grumpy
    s_Swerve.resetModulesToAbsolute();
    s_Swerve.resetTurnController();
    setAimGoal(s_Swerve.getPose());
  }

  @Override
  public void execute() {
    Pose2d pose = s_Swerve.getPose();
    setAimGoal(pose);
    boolean aligned = HubTargeting.isAligned(pose, ALIGN_TOLERANCE_DEG);

    /* Get Values, Deadband*/
    double[] driverInputs = IOConstants.getDriverInputs(driver.getController());
    boolean sticksIdle =
        Math.abs(driverInputs[0]) <= STICK_IDLE_THRESHOLD
            && Math.abs(driverInputs[1]) <= STICK_IDLE_THRESHOLD;

    /* Drive */
    // Only one of these per loop, so lockWheels() and drive() don't fight over the modules
    if (aligned && sticksIdle) {
      s_Swerve.lockWheels();
    } else {
      s_Swerve.drive(
          new Translation2d(driverInputs[0], driverInputs[1]),
          aligned ? 0.0 : s_Swerve.getTurnPidSpeed(),
          !robotCentricSup.getAsBoolean(),
          false);
    }
  }

  /** Points the turn controller at the hub center from the given pose. */
  private void setAimGoal(Pose2d pose) {
    s_Swerve.setTurnControllerGoal(Radians.of(HubTargeting.getAimHeading(pose).getRadians()));
  }

  @Override
  public void end(boolean interrupted) {
    s_Swerve.stopDriving();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
