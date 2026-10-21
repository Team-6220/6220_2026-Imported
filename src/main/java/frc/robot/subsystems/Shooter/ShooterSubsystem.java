// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.util.TunableHelper;
import frc.robot.subsystems.Vision.Cameras;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.tunable.TunableDouble;

public class ShooterSubsystem extends SubsystemBase {

  private final TelemetryTable m_shooterTelemetry = Telemetry.getTable("Shooter");
  // Motor CAN IDs
  private static final int MOTOR_41_ID = 41;
  private static final int MOTOR_34_ID = 34;
  private static final int MOTOR_9_ID = 9;
  private static final int MOTOR_31_ID = 31;
  private static final int MOTOR_35_ID = 35;

  // Motors
  /** Kicker motor (CAN ID 41) bottom */
  private final TalonFX m_motor41;

  /** Kicker motor (CAN ID 34) bottom */
  private final TalonFX m_motor34;

  /** Shooter motor (CAN ID 9) top */
  private final TalonFX m_motor9;

  /** Shooter motor (CAN ID 31) top */
  private final TalonFX m_motor31;

  /** Shooter motor (CAN ID 35) top */
  private final TalonFX m_motor35;

  // Velocity control request
  private final VelocityVoltage m_velocityRequest;

  // SysId
  private final VoltageOut m_sysIdVoltage = new VoltageOut(0);
  private final SysIdRoutine m_sysIdRoutine;

  // Tunable PID and feedforward values
  private final TunableDouble m_kp = TunableHelper.addDouble("Shooter/kP", 0.4);
  private final TunableDouble m_ki = TunableHelper.addDouble("Shooter/kI", 0.0);
  private final TunableDouble m_kd = TunableHelper.addDouble("Shooter/kD", 0.003);
  private final TunableDouble m_kv = TunableHelper.addDouble("Shooter/kV", 0.12);
  private final TunableDouble m_ks = TunableHelper.addDouble("Shooter/kS", 0.0);
  private final TunableDouble m_ka = TunableHelper.addDouble("Shooter/kA", 0.4);

  // Separate tunable RPM for top and bottom groups
  private final TunableDouble m_topTargetRPMTN =
      TunableHelper.addDouble("Shooter/TopTargetRPM", ShooterConstants.topTESTrpm);
  private final TunableDouble m_bottomTargetRPMTN =
      TunableHelper.addDouble("Shooter/BottomTargetRPM", ShooterConstants.bottomTESTrpm);
  // Tunable for the first-shot boost multiplier (do not reuse BottomTargetRPM key)
  private final TunableDouble m_firstShotBoostTN =
      TunableHelper.addDouble(
          "Shooter/FirstShotBoost", ShooterConstants.FIRST_SHOT_BOOST_MULTIPLIER);

  // Tolerance for determining if shooter is at speed (in RPS)
  private static final double VELOCITY_TOLERANCE_RPS = 1.5;

  // Current limit in amps
  private static final double CURRENT_LIMIT = 40.0;

  // First Shot Boost State Management
  /** Tracks if we are in the first shot boost phase */
  private boolean isFirstShot = true;

  /** Stores the normal (non-boosted) RPM target for current shot */
  private double normalTargetRPM = 0.0;

  /** Stores the boosted RPM target (normal * 1.05) */
  private double boostedTargetRPM = 0.0;

  /** Tracks if we've recorded the peak RPM during boost phase */
  private boolean initialRPMRecorded = false;

  /** Records the peak RPM achieved during the boost phase */
  private double peakRPM = 0.0;

  // Robot field pose (Swerve odometry + MegaTag2), used for the distance to the hub
  private final Supplier<Pose2d> m_poseSupplier;

  // Smallest and largest real distance keys in ShooterConstants.rpmAngle
  private static final double MIN_TABLE_DISTANCE_M = 0.0;
  private static final double MAX_TABLE_DISTANCE_M = 4.4;

  /**
   * @param poseSupplier the robot's field pose, used for the distance to the hub
   */
  public ShooterSubsystem(Supplier<Pose2d> poseSupplier) {
    m_poseSupplier = poseSupplier;
    CANPort shooterCANPort = CANPort.CAN_S1;
    CANBus shooterCANBus = new CANBus(shooterCANPort);
    m_motor41 = new TalonFX(MOTOR_41_ID, new CANBus(CANPort.CAN_S3));
    m_motor34 = new TalonFX(MOTOR_34_ID, new CANBus(CANPort.CAN_S3));
    m_motor9 = new TalonFX(MOTOR_9_ID, shooterCANBus);
    m_motor31 = new TalonFX(MOTOR_31_ID, shooterCANBus);
    m_motor35 = new TalonFX(MOTOR_35_ID, shooterCANBus);
    m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    configureMotors();

    // SysId routine - characterizes motor 9 (one of the top group flywheel motors)
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // default ramp rate (1 V/s)
                Volts.of(4), // dynamic step voltage (4V to prevent brownout)
                null, // default timeout (10s)
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> m_motor9.setControl(m_sysIdVoltage.withOutput(volts.in(Volts))),
                null, // Phoenix SignalLogger handles logging automatically
                this));
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Current limits
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
    config.CurrentLimits = currentLimits;

    // PID + feedforward (Slot 0)
    Slot0Configs pidConfigs = new Slot0Configs();
    pidConfigs.kP = m_kp.get();
    pidConfigs.kI = m_ki.get();
    pidConfigs.kD = m_kd.get();
    pidConfigs.kV = m_kv.get();
    pidConfigs.kS = m_ks.get();
    pidConfigs.kA = m_ka.get();
    config.Slot0 = pidConfigs;

    // Motor output config
    MotorOutputConfigs outputConfig = new MotorOutputConfigs();
    outputConfig.NeutralMode = NeutralModeValue.Coast;

    // Motor 41 -> CounterClockwise Positive
    outputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput = outputConfig;
    m_motor41.getConfigurator().apply(config);

    // Motor 1 -> Clockwise Positive (flipped)
    outputConfig.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput = outputConfig;
    m_motor34.getConfigurator().apply(config);

    // Motors 9, 31, 2 -> CounterClockwise Positive
    outputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput = outputConfig;
    m_motor9.getConfigurator().apply(config);
    m_motor31.getConfigurator().apply(config);
    m_motor35.getConfigurator().apply(config);
  }

  // ========== SysId Commands ==========

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /**
   * Runs top and bottom groups at their respective tunable RPM targets.
   *
   * <p>Implements three-phase shooting: 1. First Shot Boost: Runs at {@link
   * ShooterConstants#FIRST_SHOT_BOOST_MULTIPLIER} of target RPM 2. RPM Dip Detection: Monitors for
   * 400 RPM dip indicating shot has left 3. Normal Operation: Returns to normal RPM and operates as
   * standard
   *
   * @param topRPM Desired top-group velocity in RPM (normal, non-boosted).
   * @param controller The Xbox controller instance.
   */
  public void runAtTargetVelocity(double topRPM, CommandXboxController controller, double kicker) {
    // Follow a changed RPM target (HashShoot looks it up every loop, so it changes whenever the
    // pose distance crosses a 0.2 m table key) without restarting the first-shot boost. The boost
    // only restarts in resetShootingState(), which commands call once per button press.
    if (normalTargetRPM != topRPM) {
      normalTargetRPM = topRPM;
      boostedTargetRPM = topRPM * getBoostMultiplier();
    }

    double topRPS;
    double bottomRPS = kicker / 60.0;

    // Phase 1 & 2: Handle first shot with boost and dip detection
    if (isFirstShot) {
      double currentRPM = getAverageTopRPM();

      // Check if RPM has dipped enough to transition to normal
      if (checkForRPMDip(currentRPM)) {
        isFirstShot = false;
      }

      // Set RPM based on current phase
      if (isFirstShot) {
        // Still in boost phase
        topRPS = boostedTargetRPM / 60.0;
      } else {
        // Transitioned to normal phase
        topRPS = normalTargetRPM / 60.0;
      }
    } else {
      // Phase 3: Normal operation
      topRPS = normalTargetRPM / 60.0;
    }

    // Set motor velocities
    setTopGroupVelocityRPS(topRPS);
    if (controller.getRightTrigger() > 0.5) { // isAtSpeedFly(topRPS) &&
      setBottomGroupVelocityRPS(bottomRPS);
    }

    if (controller.getRightTrigger() < 0.5) {
      m_motor41.stopMotor();
      m_motor34.stopMotor();
    }
  }

  public void runAtTargetVelocityAuto(double topRPM) {
    // Handle new RPM target (first call or RPM changed)
    if (normalTargetRPM != topRPM) {
      normalTargetRPM = topRPM;
      isFirstShot = true;
      initialRPMRecorded = false;
      peakRPM = 0.0;
      boostedTargetRPM = topRPM * getBoostMultiplier();
    }

    double topRPS;
    double bottomRPS = m_bottomTargetRPMTN.get() / 60.0;

    // Phase 1 & 2: Handle first shot with boost and dip detection
    if (isFirstShot) {
      double currentRPM = getAverageTopRPM();

      // Check if RPM has dipped enough to transition to normal
      if (checkForRPMDip(currentRPM)) {
        isFirstShot = false;
      }

      // Set RPM based on current phase
      if (isFirstShot) {
        // Still in boost phase
        topRPS = boostedTargetRPM / 60.0;
      } else {
        // Transitioned to normal phase
        topRPS = normalTargetRPM / 60.0;
      }
    } else {
      // Phase 3: Normal operation
      topRPS = normalTargetRPM / 60.0;
    }

    // Set motor velocities
    setTopGroupVelocityRPS(topRPS);
    if (isAtSpeedFly(topRPS)) {
      setBottomGroupVelocityRPS(bottomRPS);
    }
  }

  public void runAtTargetVelocity() {
    double topRPM = m_topTargetRPMTN.get();
    if (normalTargetRPM != topRPM) {
      normalTargetRPM = topRPM;
      isFirstShot = true;
      initialRPMRecorded = false;
      peakRPM = 0.0;
      boostedTargetRPM = topRPM * ShooterConstants.FIRST_SHOT_BOOST_MULTIPLIER;
    }

    double topRPS;
    double bottomRPS = m_bottomTargetRPMTN.get() / 60.0;

    // Phase 1 & 2: Handle first shot with boost and dip detection
    if (isFirstShot) {
      double currentRPM = getAverageTopRPM();

      // Check if RPM has dipped enough to transition to normal
      if (checkForRPMDip(currentRPM)) {
        isFirstShot = false;
      }

      // Set RPM based on current phase
      if (isFirstShot) {
        // Still in boost phase
        topRPS = boostedTargetRPM / 60.0;
      } else {
        // Transitioned to normal phase
        topRPS = normalTargetRPM / 60.0;
      }
    } else {
      // Phase 3: Normal operation
      topRPS = normalTargetRPM / 60.0;
    }

    // Set motor velocities
    setTopGroupVelocityRPS(topRPS);
    if (isAtSpeedFly(topRPS)) {
      setBottomGroupVelocityRPS(bottomRPS);
    }
  }

  /** Sets kicker motors (41, 1) to the given velocity in RPS. */
  public void setBottomGroupVelocityRPS(double rps) {
    m_motor41.setControl(m_velocityRequest.withVelocity(rps));
    m_motor34.setControl(m_velocityRequest.withVelocity(rps));
  }

  /** Sets shooter motors (9, 31, 2) to the given velocity in RPS. */
  public void setTopGroupVelocityRPS(double rps) {
    m_motor9.setControl(m_velocityRequest.withVelocity(rps));
    m_motor31.setControl(m_velocityRequest.withVelocity(rps));
    m_motor35.setControl(m_velocityRequest.withVelocity(rps));
  }

  /** Sets all motors to the given velocity in RPS. */
  public void setVelocityRPS(double rps) {
    setTopGroupVelocityRPS(rps);
    setBottomGroupVelocityRPS(rps);
  }

  /** Stops all shooter motors. */
  public void stop() {
    m_motor41.stopMotor();
    m_motor34.stopMotor();
    m_motor9.stopMotor();
    m_motor31.stopMotor();
    m_motor35.stopMotor();
  }

  /** Resets the shooting state for a new shot sequence. */
  public void resetShootingState() {
    isFirstShot = true;
    normalTargetRPM = 0.0;
    boostedTargetRPM = 0.0;
    initialRPMRecorded = false;
    peakRPM = 0.0;
  }

  /**
   * Calculates the average RPM of the top three motors (9, 31, 35) for stable dip detection.
   *
   * @return Average RPM of top three motors
   */
  private double getAverageTopRPM() {
    double motor9RPM = getMotor9RPM();
    double motor31RPM = getMotor31RPM();
    double motor2RPM = getMotor2RPM();
    return (motor9RPM + motor31RPM + motor2RPM) / 3.0;
  }

  /**
   * Checks if the current RPM has dipped enough to trigger transition from boost to normal.
   *
   * @param currentRPM Current average RPM of top motors
   * @return true if RPM has dipped by at least RPM_DIP_THRESHOLD from peak
   */
  private boolean checkForRPMDip(double currentRPM) {
    if (!initialRPMRecorded) {
      // Update peak RPM while tracking the maximum
      if (currentRPM > peakRPM) {
        peakRPM = currentRPM;
      }
      // Check if we've started losing RPM (dip detected)
      if (peakRPM > 0 && (peakRPM - currentRPM) >= ShooterConstants.RPM_DIP_THRESHOLD) {
        initialRPMRecorded = true;
        return true;
      }
    }
    return false;
  }

  /** Runs all shooters at a percentage of max output (for testing). */
  public void setPercentOutput(double percent) {
    m_motor41.setThrottle(percent);
    m_motor34.setThrottle(percent);
    m_motor9.setThrottle(percent);
    m_motor31.setThrottle(percent);
    m_motor35.setThrottle(percent);
  }

  /** Runs motors 9, 2, and 31 at percent output. */
  public void setBottomGroupPercent(double percent) {
    m_motor9.setThrottle(percent);
    m_motor35.setThrottle(percent);
    m_motor31.setThrottle(percent);
  }

  /** Runs motors 41 and 1 at percent output. */
  public void setTopGroupPercent(double percent) {
    m_motor41.setThrottle(percent);
    m_motor34.setThrottle(percent);
  }

  /** Stops motors 9, 2, and 31. */
  public void stopBottomGroup() {
    m_motor9.stopMotor();
    m_motor35.stopMotor();
    m_motor31.stopMotor();
  }

  /** Stops motors 41 and 1. */
  public void stopTopGroup() {
    m_motor41.stopMotor();
    m_motor34.stopMotor();
  }

  // ========== RPM Getters ==========

  public double getMotor41RPM() {
    return m_motor41.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getMotor1RPM() {
    return m_motor34.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getMotor9RPM() {
    return m_motor9.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getMotor31RPM() {
    return m_motor31.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getMotor2RPM() {
    return m_motor35.getVelocity().getValueAsDouble() * 60.0;
  }

  /** Checks if all shooter motors are within tolerance of their target velocity. */
  public boolean isAtSpeedFly(double top) {
    double topRPS = top;
    if (topRPS == 0.0) {
      return false;
    }

    return (Math.abs(m_motor9.getVelocity().getValueAsDouble() - topRPS) < VELOCITY_TOLERANCE_RPS
        && Math.abs(m_motor31.getVelocity().getValueAsDouble() - topRPS) < VELOCITY_TOLERANCE_RPS
        && Math.abs(m_motor35.getVelocity().getValueAsDouble() - topRPS) < VELOCITY_TOLERANCE_RPS);
  }

  public double getTopTargetRPM() {
    return m_topTargetRPMTN.get();
  }

  /** Returns whether the shooter is at its current target speed. */
  public boolean isAtTargetSpeed() {
    double targetRPS = (isFirstShot ? boostedTargetRPM : normalTargetRPM) / 60.0;
    return isAtSpeedFly(targetRPS);
  }

  public double getBottomTargetRPM() {
    return m_bottomTargetRPMTN.get();
  }

  /** Updates PID values from tunable numbers (only Slot0, avoids full reconfigure). */
  private void updatePIDValues() {
    Slot0Configs pidConfigs = new Slot0Configs();
    pidConfigs.kP = m_kp.get();
    pidConfigs.kI = m_ki.get();
    pidConfigs.kD = m_kd.get();
    pidConfigs.kV = m_kv.get();
    pidConfigs.kS = m_ks.get();
    pidConfigs.kA = m_ka.get();

    m_motor41.getConfigurator().apply(pidConfigs);
    m_motor34.getConfigurator().apply(pidConfigs);
    m_motor9.getConfigurator().apply(pidConfigs);
    m_motor31.getConfigurator().apply(pidConfigs);
    m_motor35.getConfigurator().apply(pidConfigs);
  }

  /** Sets the neutral mode for all motors. */
  public void setNeutralMode(NeutralModeValue mode) {
    MotorOutputConfigs output = new MotorOutputConfigs();
    output.NeutralMode = mode;

    m_motor41.getConfigurator().apply(output);
    m_motor34.getConfigurator().apply(output);
    m_motor9.getConfigurator().apply(output);
    m_motor31.getConfigurator().apply(output);
    m_motor35.getConfigurator().apply(output);
  }

  /**
   * Returns the first-shot boost multiplier for the current distance, falling back to {@link
   * ShooterConstants#FIRST_SHOT_BOOST_MULTIPLIER} when the table entry has no boost set.
   */
  private double getBoostMultiplier() {
    // Interpolation (see ShooterConstants): return ShooterConstants.BOOST.get(getRawDist());
    Double[] entry = ShooterConstants.rpmAngle.get(getDist());
    if (entry == null || entry[2] <= 0.0) {
      return ShooterConstants.FIRST_SHOT_BOOST_MULTIPLIER;
    }
    return entry[2];
  }

  /**
   * Gets the floor-plane distance from the robot to our hub's center, rounded to the shot table's
   * 0.2 m keys and clamped to the table's range so {@code rpmAngle.get()} never returns null.
   */
  public double getDist() {
    double meters = HubTargeting.getDistanceMeters(m_poseSupplier.get());
    meters = Math.max(MIN_TABLE_DISTANCE_M, Math.min(MAX_TABLE_DISTANCE_M, meters));
    int a = (int) (meters * 100);
    double b = (double) a;
    b = b / 20.0;
    b = Math.round(b) * 2.0;
    return b / 10.0;
  }

  // For interpolation (see ShooterConstants): the unrounded, unclamped distance to the hub center.
  // The interpolating maps blend between keys and hold their end values, so no rounding is needed.
  //
  // public double getRawDist() {
  //   return HubTargeting.getDistanceMeters(m_poseSupplier.get());
  // }

  @Override
  public void periodic() {
    // Update PID if tunable numbers changed
    if (TunableHelper.consumeChanged(m_kp, m_ki, m_kd, m_kv, m_ks, m_ka)) {
      updatePIDValues();
    }
    // Update first-shot boost multiplier if it changed via tunable
    if (TunableHelper.consumeChanged(m_firstShotBoostTN)) {
      ShooterConstants.FIRST_SHOT_BOOST_MULTIPLIER = m_firstShotBoostTN.get();
    }
    m_shooterTelemetry.log("Shooter/ForwardDistance", getDist());
    Pose2d pose = m_poseSupplier.get();
    // Unrounded distance and aim error, for checking the pose against tape measurements
    m_shooterTelemetry.log("Shooter/RawHubDistance", HubTargeting.getDistanceMeters(pose));
    m_shooterTelemetry.log("Shooter/HubHeadingErrorDeg", HubTargeting.getHeadingErrorDegrees(pose));

    // RPM Telemetry
    m_shooterTelemetry.log("Shooter/Motor41RPM", getMotor41RPM());
    m_shooterTelemetry.log("Shooter/Motor1RPM", getMotor1RPM());
    m_shooterTelemetry.log("Shooter/Motor9RPM", getMotor9RPM());
    m_shooterTelemetry.log("Shooter/Motor31RPM", getMotor31RPM());
    m_shooterTelemetry.log("Shooter/Motor2RPM", getMotor2RPM());

    m_shooterTelemetry.log("Shooter/TopTargetRPM", m_topTargetRPMTN.get());
    m_shooterTelemetry.log("Shooter/BottomTargetRPM", m_bottomTargetRPMTN.get());
    m_shooterTelemetry.log("Shooter/FirstShotBoost", ShooterConstants.FIRST_SHOT_BOOST_MULTIPLIER);

    m_shooterTelemetry.log("Shooter/isFirstShot", isFirstShot);

    // Voltage Telemetry
    m_shooterTelemetry.log(
        "Shooter/Motor41Voltage", m_motor41.getMotorVoltage().getValueAsDouble());
    m_shooterTelemetry.log("Shooter/Motor1Voltage", m_motor34.getMotorVoltage().getValueAsDouble());
    m_shooterTelemetry.log("Shooter/Motor9Voltage", m_motor9.getMotorVoltage().getValueAsDouble());
    m_shooterTelemetry.log(
        "Shooter/Motor31Voltage", m_motor31.getMotorVoltage().getValueAsDouble());
    m_shooterTelemetry.log("Shooter/Motor2Voltage", m_motor35.getMotorVoltage().getValueAsDouble());

    // Current Telemetry
    m_shooterTelemetry.log(
        "Shooter/Motor41Current", m_motor41.getSupplyCurrent().getValueAsDouble());
    m_shooterTelemetry.log(
        "Shooter/Motor1Current", m_motor34.getSupplyCurrent().getValueAsDouble());
    m_shooterTelemetry.log("Shooter/Motor9Current", m_motor9.getSupplyCurrent().getValueAsDouble());
    m_shooterTelemetry.log(
        "Shooter/Motor31Current", m_motor31.getSupplyCurrent().getValueAsDouble());
    m_shooterTelemetry.log(
        "Shooter/Motor2Current", m_motor35.getSupplyCurrent().getValueAsDouble());

    // LED Telemetry (Shooter Status, Debug Purposes; temporary)
    m_shooterTelemetry.log("Shooter/FlywheelsReady", isAtSpeedFly(getTopTargetRPM() / 60.0));
    m_shooterTelemetry.log("Shooter/TargetVisible", Cameras.FRONT.hasTarget());
    m_shooterTelemetry.log("Shooter/LimelightTX", Cameras.FRONT.getTXDegrees());
  }
}
