package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.IOConstants;
import frc.robot.subsystems.Drive.Swerve;
import java.util.function.BooleanSupplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Translation2d;

/** Swerve drive command used for teleop period. */
public class SwerveCom extends Command {
  private Swerve s_Swerve;
  private BooleanSupplier robotCentricSup;
  private CommandNiDsXboxController driver;

  public SwerveCom(Swerve s_Swerve, CommandNiDsXboxController driver, BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    this.driver = driver;
    this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void initialize() {
    s_Swerve.setIsAuto(RobotState.isAutonomous());
    // Initialize so that the swerve doesn't become grumpy
    s_Swerve.resetModulesToAbsolute();
  }

  @Override
  public void execute() {
    if (!RobotState.isAutonomous()) {
      /* Get Values, Deadband*/
      double[] driverInputs = IOConstants.getDriverInputs(driver.getNiDsXboxController());
      boolean robotCentric = robotCentricSup.getAsBoolean();
      // getDriverInputs flips translation on red for field-relative driving; undo that when
      // driving robot-relative so forward on the stick is always the robot's front
      if (robotCentric && Constants.isRed.equals("red")) {
        driverInputs[0] *= -1;
        driverInputs[1] *= -1;
      }
      /* Drive */
      s_Swerve.drive(
          new Translation2d(driverInputs[0], driverInputs[1]), driverInputs[2], !robotCentric, true);
    }
  }
}
