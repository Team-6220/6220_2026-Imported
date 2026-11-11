package frc.robot.commands.Autos;

import frc.robot.commands.AlignAndMove;
import frc.robot.commands.HashShootAuto;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Intake.BeltSubsystem;
import frc.robot.subsystems.Shooter.AnglerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.PrintCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.button.CommandXboxController;

/** Starts at the trench, lines up on the goal, and shoots the preload. */
public class TrenchPreloadAuto extends SequentialCommandGroup {

  public TrenchPreloadAuto(
      Swerve s_swerve,
      AnglerSubsystem m_angler,
      ShooterSubsystem m_shooter,
      BeltSubsystem m_belt,
      CommandXboxController controller) {
    addRequirements(s_swerve, m_angler, m_belt, m_shooter);
    addCommands(
        new PrintCommand("starting trench preload auto"),
        new InstantCommand(() -> s_swerve.zeroHeading(), s_swerve),
        new AlignAndMove(s_swerve, controller, controller.rightBumper()).withTimeout(2),
        new HashShootAuto(m_angler, m_shooter, m_belt).withTimeout(4),
        new PrintCommand("done"));
  }
}
