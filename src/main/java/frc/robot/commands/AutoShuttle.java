// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake.BeltSubsystem;
import frc.robot.subsystems.Shooter.AnglerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandNiDsXboxController;

/**
 * Spins up for a pass to the shuttle target (see HubTargeting.getShuttleTarget()), picking RPM and
 * angle from ShooterConstants.shuttleRpmAngle by pose distance. RT feeds, the same as HashShoot.
 * Pair with AlignAndMove aimed at the shuttle target.
 */
public class AutoShuttle extends Command {
  AnglerSubsystem m_angler;

  ShooterSubsystem m_shoot;

  BeltSubsystem m_belt;
  CommandNiDsXboxController controller;

  public AutoShuttle(
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
    // Looked up every loop so RPM and angle follow the pose while the driver moves
    double[] shot = m_shoot.getShot(true);
    m_angler.setAngle(shot[1]);
    m_shoot.runAtTargetVelocity(shot[0], controller, 1200, shot[2]);
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
