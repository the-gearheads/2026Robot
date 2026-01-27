package frc.robot.subsystems.swerve;

import static frc.robot.constants.SwerveConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.OptionalDouble;
import java.util.Queue;
import java.util.function.DoubleSupplier;

public class ModuleIOSpark implements ModuleIO {
  // Drive motor components
  private final SparkFlex drive;
  private final RelativeEncoder driveEncoder;
  private final SparkClosedLoopController drivePid;
  private final SparkFlexConfig driveConfig = new SparkFlexConfig();

  // Steer motor components
  private final SparkMax steer;
  private final SparkAbsoluteEncoder steerEncoder;
  private final SparkClosedLoopController steerPid;
  private final SparkMaxConfig steerConfig = new SparkMaxConfig();

  private final String modulePath;
  private final int id;
  private final Rotation2d offset;

  private double driveSetpoint = 0;
  private Rotation2d targetSteerAngle = Rotation2d.kZero;
  private boolean manualVoltageOnly = false;

  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  public ModuleIOSpark(int id, Rotation2d offset, String modulePath) {
    this.id = id;
    this.offset = offset;
    this.modulePath = modulePath;

    // Initialize drive motor
    drive = new SparkFlex(MOTOR_IDS[id][0], MotorType.kBrushless);
    driveEncoder = drive.getEncoder();
    drivePid = drive.getClosedLoopController();

    // Initialize steer motor
    steer = new SparkMax(MOTOR_IDS[id][1], MotorType.kBrushless);
    steerEncoder = steer.getAbsoluteEncoder();
    steerPid = steer.getClosedLoopController();

    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(drive, driveEncoder::getPosition);
    turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(steer, steerEncoder::getPosition);
  }

  public void configure() {
    configureDrive();
    configureSteer();
  }

  private void configureDrive() {
    drive.setCANTimeout(250);
    driveConfig.smartCurrentLimit(DRIVE_CURRENT_LIMIT);
    driveConfig.idleMode(IdleMode.kBrake);
    driveConfig.inverted(IS_INVERTED[id]);
    driveConfig.voltageCompensation(12);
    driveConfig.encoder.positionConversionFactor(DRIVE_POS_FACTOR);
    driveConfig.encoder.velocityConversionFactor(DRIVE_VEL_FACTOR);
    driveConfig.closedLoop.p((DRIVE_PID[0] / 12.0) * DRIVE_VEL_FACTOR);
    driveConfig.closedLoop.i(0);
    driveConfig.closedLoop.d(0);
    driveConfig.closedLoop.feedForward.kS(DRIVE_FEEDFORWARD.getKs());
    driveConfig.closedLoop.feedForward.kA(DRIVE_FEEDFORWARD.getKa());
    driveConfig.closedLoop.feedForward.kV(DRIVE_FEEDFORWARD.getKv());

    driveConfig.signals.appliedOutputPeriodMs(20);
    driveConfig.signals.primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY));
    driveConfig.signals.primaryEncoderVelocityPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY));

    drive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    drive.setCANTimeout(0);
  }

  private void configureSteer() {
    steer.setCANTimeout(250);
    steerConfig.smartCurrentLimit(STEER_CURRENT_LIMIT);
    steerConfig.idleMode(IdleMode.kBrake);

    steerConfig.absoluteEncoder.positionConversionFactor(STEER_POS_FACTOR);
    steerConfig.absoluteEncoder.velocityConversionFactor(STEER_VEL_FACTOR);
    steerConfig.absoluteEncoder.inverted(true);

    steerConfig.closedLoop.positionWrappingEnabled(true);
    steerConfig.closedLoop.positionWrappingMinInput(0);
    steerConfig.closedLoop.positionWrappingMaxInput(Math.PI * 2);

    steerConfig.closedLoop.pid(STEER_PIDF[0], STEER_PIDF[1], STEER_PIDF[2]);
    steerConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    steerConfig.signals.appliedOutputPeriodMs(20);
    steerConfig.signals.primaryEncoderPositionAlwaysOn(false);
    steerConfig.signals.primaryEncoderVelocityPeriodMs(40);

    steerConfig.signals.absoluteEncoderPositionAlwaysOn(true);
    steerConfig.signals.absoluteEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY));
    steerConfig.signals.absoluteEncoderVelocityAlwaysOn(true);
    steerConfig.signals.absoluteEncoderVelocityPeriodMs(20);

    steer.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    steer.setCANTimeout(0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(drive, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(drive, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        drive,
        new DoubleSupplier[] { drive::getAppliedOutput, drive::getBusVoltage },
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(drive, drive::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        steer,
        steerEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(offset));
    ifOk(steer, steerEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        steer,
        new DoubleSupplier[] { steer::getAppliedOutput, steer::getBusVoltage },
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(steer, steer::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions = turnPositionQueue.stream()
        .map((Double value) -> new Rotation2d(value).minus(offset))
        .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    drive.setVoltage(volts);
  }

  @Override
  public void setSteerVoltage(double volts) {
    steer.setVoltage(volts);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts = DRIVE_FEEDFORWARD.calculate(velocityRadPerSec);
    drivePid.setSetpoint(
        velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setSteerAngle(Rotation2d rotation) {
    double setpoint = rotation.plus(offset).getRadians();
    steerPid.setSetpoint(setpoint, ControlType.kPosition);
  }
}