// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake.BeltSubsystem;
import frc.robot.subsystems.Shooter.AnglerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.telemetry.Telemetry;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HashShoot extends Command {
  /** Creates a new HashShoot. */
  AnglerSubsystem m_angler;

  ShooterSubsystem m_shoot;

  BeltSubsystem m_belt;
  CommandNiDsXboxController controller;
  double degrees;
  double rpm;

  public HashShoot(
      AnglerSubsystem m_angler,
      ShooterSubsystem m_shoot,
      BeltSubsystem m_belt,
      CommandNiDsXboxController controller) {
    this.m_angler = m_angler;
    this.m_shoot = m_shoot;
    this.m_belt = m_belt;
    this.controller = controller;
    addRequirements(m_angler, m_shoot, m_belt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset shooter state for new shot sequence (enables first shot boost)
    m_shoot.resetShootingState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Looked up every loop so RPM and angle follow the pose while the driver moves.
    // If shooting is jittery (setpoints jumping as the distance crosses 0.2 m table keys), turn on
    // ShooterConstants.INTERPOLATION_ENABLED, or move this lookup back into initialize().
    double[] shot = m_shoot.getShot(false);
    rpm = shot[0];
    degrees = shot[1];

    m_angler.setAngle(degrees);
    m_shoot.runAtTargetVelocity(rpm, controller, 1200, shot[2], false);
    m_belt.simpleDrive(-0.5);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_angler.setAngle(0);
    m_shoot.stop();
    m_belt.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
