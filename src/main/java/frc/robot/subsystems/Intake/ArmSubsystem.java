// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.lib.util.TunableHelper;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.tunable.TunableDouble;

public class ArmSubsystem extends SubsystemBase {

  private static ArmSubsystem INSTANCE = null;

  private final TelemetryTable m_armtelemetry = Telemetry.getTable("Arm");

  // Non-gain tunables (kept as TunableDouble)
  private final TunableDouble ArmIZone =
      TunableHelper.addDouble("arm izone", ArmConstants.armIZone);
  private final TunableDouble ArmTolerance =
      TunableHelper.addDouble("arm tolerance", ArmConstants.armTolerance);
  private final TunableDouble ArmMaxVel =
      TunableHelper.addDouble("arm max vel", ArmConstants.armMaxVel);
  private final TunableDouble ArmMaxAccel =
      TunableHelper.addDouble("arm max accel", ArmConstants.armMaxAccel);
  private final TunableDouble ArmIdleVoltage =
      TunableHelper.addDouble("arm idle voltage", ArmConstants.armIdleVoltage);
  private final TunableDouble ArmVoltage =
      TunableHelper.addDouble("arm voltage", ArmConstants.armVoltage);

  // Gain tunables — editable on the dashboard, applied in periodic() when changed
  private final TunableDouble ArmKp = TunableHelper.addDouble("arm_tune_kP", ArmConstants.armKp);
  private final TunableDouble ArmKi = TunableHelper.addDouble("arm_tune_kI", ArmConstants.armKi);
  private final TunableDouble ArmKd = TunableHelper.addDouble("arm_tune_kD", ArmConstants.armKd);
  private final TunableDouble ArmKs = TunableHelper.addDouble("arm_tune_kS", ArmConstants.armKs);
  private final TunableDouble ArmKg = TunableHelper.addDouble("arm_tune_kG", ArmConstants.armKg);
  private final TunableDouble ArmKv = TunableHelper.addDouble("arm_tune_kV", ArmConstants.armKv);

  private double idleOutVolt = ArmConstants.armIdleVoltage;
  private double intakeOutVolt = ArmConstants.armVoltage;

  private final ProfiledPIDController m_Controller;
  private ArmFeedforward m_Feedforward;
  private TrapezoidProfile.Constraints m_Constraints;
  private double feedForwardOutput, PIDOutput;
  private double lastUpdate = 0;

  private final SparkMax armMotor;
  private final SparkMaxConfig armMotorConfig = new SparkMaxConfig();
  private final RelativeEncoder armEncoder;

  private static final class ArmConstants {
    public static final int armMotorID = 18;

    public static final int stallLimit = 30;
    public static final int freeLimit = 30;

    // Inverted so the arm's down direction is positive (motor and built-in encoder flip together).
    // Measured: top to bottom read -24.36 motor rotations before this was inverted.
    public static final boolean armInvert = true;
    public static final IdleMode armIdleMode = IdleMode.kBrake;

    // Total reduction from the NEO to the arm, measured: top (straight up) to bottom (horizontal)
    // is ~90 deg and took 27.19 motor rotations, so 27.19 * 360 / 90 = ~108.8:1 (50/14 sprockets
    // times ~30:1 more). REVLib 2027 has no encoder conversion factor, so getPosition() converts
    // motor rotations to arm degrees itself. Refine if the 90 deg estimate turns out off.
    public static final double gearRatio = 27.19 * 360.0 / 90.0;
    public static final double degreesPerMotorRotation = 360.0 / gearRatio;

    // Arm angles are relative to the top: 0 at the top, positive going down. The gravity
    // feedforward needs to know where horizontal is, so this is the top's angle above horizontal.
    // ASSUMPTION: the top is straight up. Change this if the top position isn't vertical.
    public static final double topAngleAboveHorizontalDeg = 90.0;

    // Gains are per degree of arm. They used to be per motor rotation (3.3 deg each), so the old
    // kP of 0.45 is 0.45 / 3.31 = ~0.136 per degree.
    public static final double armKp = 0.136;
    public static final double armKi = 0.0;
    public static final double armKd = 0.0;
    public static final double armKa = 0.0;
    public static final double armKg = 0.3; // volts to hold the arm level
    public static final double armKv = 2.2; // volts per rad/s: 12 V / (NEO free speed / gearRatio)
    public static final double armKs = 0.0;
    public static final double armIZone = 5; // degrees
    public static final double armTolerance = 2; // degrees
    public static final double armMaxVel = 200; // degrees per second
    public static final double armMaxAccel = 800; // degrees per second squared

    public static final double armIdleVoltage = 0.0;
    public static final double armVoltage = 5;

    // Goal limits in arm degrees (0 = top, positive = down, 90 = horizontal/bottom)
    public static final double maxDegrees = 95;
    public static final double minDegrees = 0;
  }

  public ArmSubsystem() {
    CANPort armCANPort = CANPort.CAN_S2;
    armMotor = new SparkMax(armCANPort, ArmConstants.armMotorID, MotorType.kBrushless);
    armMotorConfig.inverted(ArmConstants.armInvert);
    armMotorConfig.smartCurrentLimit(ArmConstants.stallLimit, ArmConstants.freeLimit);
    armMotorConfig.idleMode(ArmConstants.armIdleMode);
    armMotor.configure(
        armMotorConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    armEncoder = armMotor.getEncoder();

    m_Constraints =
        new TrapezoidProfile.Constraints(ArmConstants.armMaxVel, ArmConstants.armMaxAccel);
    m_Controller =
        new ProfiledPIDController(
            ArmConstants.armKp, ArmConstants.armKi, ArmConstants.armKd, m_Constraints);
    m_Feedforward =
        new ArmFeedforward(
            ArmConstants.armKs, ArmConstants.armKg, ArmConstants.armKv, ArmConstants.armKa);
    m_Controller.setIZone(ArmIZone.get());
    m_Controller.setTolerance(ArmTolerance.get());
  }

  @Override
  public void periodic() {
    // --- Live gain tuning via Tunables ---
    // If P, I, or D changed, push update to controller
    if (TunableHelper.consumeChanged(ArmKp, ArmKi, ArmKd)) {
      m_Controller.setPID(ArmKp.get(), ArmKi.get(), ArmKd.get());
      System.out.println(
          "[Arm] PID updated -> P:" + ArmKp.get() + " I:" + ArmKi.get() + " D:" + ArmKd.get());
    }

    // If Ks, Kg, or Kv changed, rebuild feedforward
    if (TunableHelper.consumeChanged(ArmKs, ArmKg, ArmKv)) {
      m_Feedforward = new ArmFeedforward(ArmKs.get(), ArmKg.get(), ArmKv.get(), ArmConstants.armKa);
      System.out.println(
          "[Arm] FF updated -> Ks:" + ArmKs.get() + " Kg:" + ArmKg.get() + " Kv:" + ArmKv.get());
    }

    // --- Other tunable updates ---
    if (TunableHelper.consumeChanged(ArmIZone)) {
      m_Controller.setIZone(ArmIZone.get());
    }

    if (TunableHelper.consumeChanged(ArmTolerance)) {
      m_Controller.setTolerance(ArmTolerance.get());
    }

    if (TunableHelper.consumeChanged(ArmMaxVel, ArmMaxAccel)) {
      m_Constraints = new TrapezoidProfile.Constraints(ArmMaxVel.get(), ArmMaxAccel.get());
      m_Controller.setConstraints(m_Constraints);
    }

    // --- Telemetry ---

    // Position & controller state
    m_armtelemetry.log("position", getPosition());
    m_armtelemetry.log("goal", m_Controller.getGoal().position);
    m_armtelemetry.log("setpoint_pos", m_Controller.getSetpoint().position);
    m_armtelemetry.log("setpoint_vel", m_Controller.getSetpoint().velocity);
    m_armtelemetry.log("position_error", m_Controller.getPositionError());
    m_armtelemetry.log("velocity_error", m_Controller.getVelocityError());
    m_armtelemetry.log("at_setpoint", m_Controller.atSetpoint());
    m_armtelemetry.log("at_goal", m_Controller.atGoal());

    // Controller outputs
    m_armtelemetry.log("ff_output", feedForwardOutput);
    m_armtelemetry.log("pid_output", PIDOutput);
    m_armtelemetry.log("total_output", feedForwardOutput + PIDOutput);

    // Motor diagnostics
    m_armtelemetry.log(
        "motor_output_volts", armMotor.getBusVoltage().get() * armMotor.getAppliedOutput().get());
    m_armtelemetry.log("motor_current_amps", armMotor.getOutputCurrent());
    m_armtelemetry.log("motor_temp_celsius", armMotor.getMotorTemperature());
  }

  public void setGoal(double goal) {
    resetPID();
    if (goal > ArmConstants.maxDegrees) goal = ArmConstants.maxDegrees;
    if (goal < ArmConstants.minDegrees) goal = ArmConstants.minDegrees;
    m_Controller.setGoal(goal);
  }

  public void swingToGoal() {
    // ArmFeedforward expects 0 = horizontal and positive voltage moving the angle up. Here 0 is the
    // top and positive voltage moves the arm down (positive degrees), so shift into a frame where
    // positive voltage still increases the angle: 180 - top + position. Holding level (position =
    // top angle) then gives cos(180) = -1, i.e. a negative (lifting) voltage of kG.
    double ffAngleDeg =
        180.0 - ArmConstants.topAngleAboveHorizontalDeg + m_Controller.getSetpoint().position;
    feedForwardOutput =
        m_Feedforward.calculate(
            Math.toRadians(ffAngleDeg), Math.toRadians(m_Controller.getSetpoint().velocity));

    lastUpdate = Timer.getTimestamp();
    PIDOutput = m_Controller.calculate(getPosition());

    // System.out.println("pos=" + getPosition() + " goal=" + m_Controller.getGoal().position
    // + " pid=" + PIDOutput + " ff=" + feedForwardOutput);

    armMotor.setVoltage(PIDOutput + feedForwardOutput);
  }

  public void resetEncoder() {
    armEncoder.setPosition(0);
  }

  /**
   * Tells the relative encoder where the arm is right now. It only knows positions relative to
   * where it was at power-on, so use this when the arm is at a known spot (e.g. the top).
   */
  public void setEncoderPosition(double degrees) {
    armEncoder.setPosition(degrees / ArmConstants.degreesPerMotorRotation);
  }

  public void resetPID() {
    m_Controller.reset(getPosition());
  }

  /** Arm angle in degrees: 0 at the top, positive going down. */
  public double getPosition() {
    return armEncoder.getPosition().get() * ArmConstants.degreesPerMotorRotation;
  }

  public boolean controllerAtGoal() {
    return m_Controller.atGoal();
  }

  public void simpleDriveArm(double motorOutput) {
    armMotor.setVoltage(motorOutput * 8);
  }

  public void maintain() {
    if (TunableHelper.consumeChanged(ArmIdleVoltage)) idleOutVolt = ArmIdleVoltage.get();
    armMotor.setVoltage(-idleOutVolt);
  }

  public void setMaxVel(double maxVel) {
    ArmMaxVel.set(maxVel);
    m_armtelemetry.log("max vel", maxVel);
  }

  public void setMaxAccel(double maxAccel) {
    ArmMaxAccel.set(maxAccel);
    m_armtelemetry.log("max accel", maxAccel);
  }

  public void stopDriving() {
    armMotor.stopMotor();
  }

  public static synchronized ArmSubsystem getInstance() {
    if (INSTANCE == null) INSTANCE = new ArmSubsystem();
    return INSTANCE;
  }
}
