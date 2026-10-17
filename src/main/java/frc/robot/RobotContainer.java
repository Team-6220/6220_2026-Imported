// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import java.util.function.Supplier;
import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.Joystick;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.button.Trigger;
import frc.robot.commands.AlignAndFlywheels;
import frc.robot.commands.AlignAndMove;
import frc.robot.commands.HashShootAuto;
import frc.robot.commands.Autos.FollowPathAuto;
import frc.robot.commands.ArmToPositionCommand;
import frc.robot.commands.ArmUpAndDown;
import frc.robot.commands.Autos.BasicAutoBlue;
import frc.robot.commands.Autos.BasicAutoRed;
import frc.robot.commands.Autos.BasicAuto;
import frc.robot.commands.ManualArm;
import frc.robot.commands.PassToAlliance;
import frc.robot.commands.SwerveCom;
import frc.robot.commands.TestRollerCommand;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Intake.ArmSubsystem;
import frc.robot.subsystems.Intake.BeltSubsystem;
import frc.robot.subsystems.Intake.RollerSubsystem;
import frc.robot.subsystems.LEDs.AdressableLEDs;
import frc.robot.subsystems.Shooter.AnglerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Vision.Cameras;
import org.wpilib.driverstation.DriverStation;
import frc.robot.commands.Autos.SamAuto.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final Selectable<Supplier<Command>> autoChooser = new Selectable<>();

  /* Subsystems */
  private final AdressableLEDs s_LED = new AdressableLEDs();
  private final Swerve s_Swerve = new Swerve();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final AnglerSubsystem m_angler = new AnglerSubsystem();
  private final ArmSubsystem arm = ArmSubsystem.getInstance();
  private final BeltSubsystem belt = BeltSubsystem.getInstance();
  private final RollerSubsystem roller = RollerSubsystem.getInstance();
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final Joystick m_joystick = new Joystick(1);

  private final GenericHID m_buttonBoard = DriverStation.getGenericHID(2);

  private final TelemetryTable m_shooterTelemetry = Telemetry.getTable("Shooter");
  private final TelemetryTable m_visionTelemetry = Telemetry.getTable("Vision");

  private static final double ALIGN_TOLERANCE_DEG = 2.0;
  private static final double MAX_SHOOT_DISTANCE_M = 4.0;
  // Reverse kicker to clear jammed balls. Kicker normally feeds at ~1200 RPM (~20% of free speed),
  // so reverse harder than that to push a stuck ball back out.
  private static final double KICKER_REVERSE_PERCENT = 0.5;
  // Arm setpoints (same values as the arm0 / arm90 button bindings below)
  private static final double ARM_DOWN = -2;
  private static final double ARM_UP = -29.785476684570312;

  public RobotContainer() {
    // Initialize climber subsystem based on robot mode

    // Configure the trigger bindings
    s_Swerve.configureAutoBuilder();
    s_Swerve.zeroHeading(m_driverController.getController());

    s_Swerve.setDefaultCommand(
        new SwerveCom(s_Swerve, m_driverController, m_driverController.leftBumper()));

    arm.setDefaultCommand(new ManualArm(m_joystick));

    // Must be registered before any PathPlanner path or auto is loaded
    registerNamedCommands();

    // Autos are built when auto starts (not here) so they see the current alliance
    // (e.g. BasicAuto reads Constants.isRed in its constructor).
    autoChooser.add("Do nothing", InstantCommand::new);
    autoChooser.addDefault(
        "basicAuto", () -> new BasicAuto(s_Swerve, m_angler, m_shooter, belt, m_driverController));
    autoChooser.add(
        "Red", () -> new BasicAutoRed(s_Swerve, m_angler, m_shooter, belt, m_driverController));
    autoChooser.add(
        "Blue", () -> new BasicAutoBlue(s_Swerve, m_angler, m_shooter, belt, m_driverController));
    autoChooser.add("samautov1", () -> new SamAutoV1(s_Swerve));
    autoChooser.add("samautov2", () -> new SamAutoV2(s_Swerve));
    autoChooser.add(
        "brightAuto",
        () -> new BrightTryMakeAuto(s_Swerve, arm, m_driverController, m_angler, m_shooter, belt));
    autoChooser.add("stationToShoot", () -> new FollowPathAuto(s_Swerve, "stationtoshoot"));
    // Every .auto made in PathPlanner (deploy/pathplanner/autos/) shows up as "PP: <name>"
    for (String autoName : AutoBuilder.getAllAutoNames()) {
      autoChooser.add("PP: " + autoName, () -> new PathPlannerAuto(autoName));
    }
    // Published to NT at /Tunables/Auto Chooser (Elastic "ComboBox Chooser" widget)
    Tunables.publish("Auto Chooser", autoChooser);

    configureBindings();
  }

  /**
   * Commands PathPlanner can run by name, from "Named Command" blocks in a .auto or event markers
   * on a path. The name typed in PathPlanner must match exactly (case-sensitive).
   *
   * <p>Commands that never end ("intake", "outtake", "shoot") should go in an event marker zone
   * (runs while the robot is in that part of the path), or in a deadline/race group in a .auto.
   * "align" and "alignAndShoot" drive the robot, so only use them between paths, not as markers.
   */
  private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "intake",
        Commands.parallel(new TestRollerCommand(true), new ArmToPositionCommand(arm, ARM_DOWN)));
    NamedCommands.registerCommand("outtake", new TestRollerCommand(false));
    NamedCommands.registerCommand("armDown", new ArmToPositionCommand(arm, ARM_DOWN));
    NamedCommands.registerCommand("armUp", new ArmToPositionCommand(arm, ARM_UP));
    NamedCommands.registerCommand("shoot", new HashShootAuto(m_angler, m_shooter, belt));
    NamedCommands.registerCommand(
        "align",
        new AlignAndMove(s_Swerve, m_driverController, m_driverController.rightBumper())
            .withTimeout(2));
    NamedCommands.registerCommand(
        "alignAndShoot",
        new AlignAndMove(s_Swerve, m_driverController, m_driverController.rightBumper())
            .withTimeout(2)
            .andThen(new HashShootAuto(m_angler, m_shooter, belt).withTimeout(4)));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * org.wpilib.command2.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link org.wpilib.command2.button.CommandPS4Controller
   * PS4} controllers or {@link org.wpilib.command2.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    Trigger angleUp = new Trigger(() -> m_joystick.getRawButton(10));
    Trigger angleDown = new Trigger(() -> m_joystick.getRawButton(11));
    Trigger passMid = m_driverController.rightBumper();
    Trigger passFar = m_driverController.b();
    Trigger resetEncoder = new Trigger(() -> m_joystick.getRawButton(6));
    Trigger manualArm = new Trigger(() -> m_joystick.getRawButton(4));
    Trigger joystickIntake = new Trigger(() -> m_joystick.getRawButton(0));
    Trigger joystickOuttake = new Trigger(() -> m_joystick.getRawButton(1));

    // Trigger armUpAndDown = new Trigger(() -> m_buttonBoard.getRawButton(3));
    // Trigger arm0 = new Trigger(() -> m_buttonBoard.getRawButton(5));
    // Trigger arm90 = new Trigger(() -> m_buttonBoard.getRawButton(6));
    // Trigger armReset = new Trigger(() -> m_joystick.getRawButton(7));

    resetEncoder.onTrue(Commands.runOnce(() -> m_angler.resetEncoder()));
    // armReset.onTrue(Commands.runOnce(() -> arm.resetEncoder()));

    // armUpAndDown.whileTrue(new ArmUpAndDown(arm));

    m_driverController
        .y()
        .onTrue(new InstantCommand(() -> s_Swerve.zeroHeading(m_driverController.getController())));

    angleDown.whileTrue(
        Commands.runEnd(() -> m_angler.setSpeed(-0.15), () -> m_angler.stop(), m_angler));
    angleUp.whileTrue(
        Commands.runEnd(() -> m_angler.setSpeed(0.15), () -> m_angler.stop(), m_angler));
    manualArm.onTrue(new ManualArm(m_joystick));

    m_driverController
        .a()
        .onTrue(
            Commands.run(() -> m_angler.setAngle(0), m_angler).until(() -> m_angler.isAtAngle(0)));

    // m_driverController.x().whileTrue(new ShooterTESTER(m_shooter, belt));

    passMid.whileTrue(new PassToAlliance(m_angler, m_shooter, belt, m_driverController, 2200, 1200, 32, -0.5));

    passFar.whileTrue(new PassToAlliance(m_angler, m_shooter, belt, m_driverController, 3100, 1200, 32, -0.5));

    m_driverController.x().whileTrue(new PassToAlliance(m_angler, m_shooter, belt, m_driverController, -2000, -2400, 0, 0.5));

    joystickIntake.whileTrue(new TestRollerCommand(true));

    joystickOuttake.whileTrue(new TestRollerCommand(false));

    // // Kicker motors (41, 34) are driven by setTopGroupPercent / stopTopGroup
    // reverseKicker.whileTrue(
    //     Commands.runEnd(
    //         () -> m_shooter.setTopGroupPercent(-KICKER_REVERSE_PERCENT),
    //         () -> m_shooter.stopTopGroup(),
    //         m_shooter));

    m_driverController
        .leftTrigger()
        .whileTrue(new AlignAndFlywheels(s_Swerve, m_driverController, m_angler, m_shooter, belt));

    // arm0.onTrue(new ArmToPositionCommand(arm, -2));

    // arm90.onTrue(new ArmToPositionCommand(arm, -29.785476684570312));

    // ==================== State-Based LED Controls ====================

    // --- Individual state triggers ---
    Trigger flywheelsReady = new Trigger(() -> m_shooter.isAtTargetSpeed());

    Trigger aligned =
        new Trigger(
            () ->
                Math.abs(Cameras.FRONT.getTXDegrees()) < ALIGN_TOLERANCE_DEG
                    && Cameras.FRONT.hasTarget());

    Trigger inRange =
        new Trigger(
            () -> {
              double dist = m_shooter.getDist();
              return dist > 0 && dist <= MAX_SHOOT_DISTANCE_M;
            });

    // Stable lock for driver rumble to avoid flicker/noise
    Trigger targetLocked = aligned.and(inRange).debounce(0.1);

    // --- Combined "ready to shoot" trigger ---
    // We require everything (speed, alignment, range) for the final triggers so rumble corresponds
    // to the actual firing window.
    Trigger readyToShoot = flywheelsReady.and(aligned);

    Trigger isShooting = m_driverController.rightTrigger();

    // --- LED bindings (lowest to highest priority) made mutually exclusive ---

    // Aligned but not at speed -> Flashing Green
    aligned
        .and(flywheelsReady.negate())
        .and(isShooting.negate())
        .whileTrue(s_LED.runPattern(s_LED.blink(s_LED.solidGreen(), 0.15)));

    // At speed but not aligned OR actively shooting while not aligned -> Flashing Orange
    flywheelsReady
        .or(isShooting)
        .and(aligned.negate())
        .whileTrue(s_LED.runPattern(s_LED.blink(s_LED.solidDarkOrange(), 0.15)));

    // Aligned and at speed (but not shooting) -> Solid Green
    readyToShoot.and(isShooting.negate()).whileTrue(s_LED.runPattern(s_LED.solidGreen()));

    // Actively shooting AND aligned -> Flashing Red
    isShooting.and(aligned).whileTrue(s_LED.runPattern(s_LED.blink(s_LED.solidRed(), 0.15)));

    // Haptic feedback (rumble): Pulse when target is aligned + in range (before full spin-up).
    readyToShoot.onTrue(
        Commands.runOnce(
                () -> {
                  m_driverController.getHID().setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, 1.0);
                  m_driverController.getHID().setRumble(GenericHID.RumbleType.LEFT_RUMBLE, 1.0);
                })
            .andThen(Commands.waitSeconds(0.25))
            .andThen(
                Commands.runOnce(
                    () -> {
                      m_driverController.getHID().setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, 0.0);
                      m_driverController.getHID().setRumble(GenericHID.RumbleType.LEFT_RUMBLE, 0.0);
                    })));
  }

  /** Publish shooter booleans needed by Elastic dashboard widgets. */
  public void publishDriverDashboardBooleans() {
    boolean aligned =
        Cameras.FRONT.hasTarget()
            && Math.abs(Cameras.FRONT.getTXDegrees()) < ALIGN_TOLERANCE_DEG;
    double dist = m_shooter.getDist();
    boolean shortRange = dist > 0 && dist <= MAX_SHOOT_DISTANCE_M;

    m_shooterTelemetry.log("Shooter/Aligned", aligned);
    m_shooterTelemetry.log("Shooter/FlywheelsAtTarget", m_shooter.isAtTargetSpeed());
    m_shooterTelemetry.log("Shooter/ShortRange", shortRange);

    m_visionTelemetry.log("FrontPipelineNonZero", Cameras.isFrontPipelineNonZero());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Supplier<Command> selected = autoChooser.getSelected();
    Command auto = selected != null ? selected.get() : null;
    System.out.println("Auto: " + (auto != null ? auto.getName() : "none"));
    return auto;
  }
}
