package frc.robot.subsystems.swerve.motors;

import static frc.robot.constants.SwerveConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import java.util.OptionalDouble;

public class DriveMotor {

  SparkFlex flex;
  SparkFlexConfig config = new SparkFlexConfig();
  RelativeEncoder encoder;

  SparkClosedLoopController pid;

  String modulePath;
  boolean manualVoltageOnly = false;
  double driveSetpoint = 0;

  int index;

  /* id is the CAN id, index is the index into the array of modules and stuff */
  public DriveMotor(int id, int index, String modulePath) {
    flex = new SparkFlex(id, MotorType.kBrushless);
    pid = flex.getClosedLoopController();
    encoder = flex.getEncoder();
    this.modulePath = modulePath;
    this.index = index;
  }
  
  public void log() {
    Logger.recordOutput(modulePath + "/driveVolts", getVoltage());
    Logger.recordOutput(modulePath + "/drivePos", getPosition());
    Logger.recordOutput(modulePath + "/driveVel", getVelocity());
    Logger.recordOutput(modulePath + "/targetSpeed", driveSetpoint);
    Logger.recordOutput(modulePath + "/manualVoltageOnly", manualVoltageOnly);
  }

  public void setSpeed(double speed) {
    driveSetpoint = speed;
    pid.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public double getVoltage() {
    return flex.getAppliedOutput() * flex.getBusVoltage();
  }

  public void periodic() {
  }

  /* meant for sysid and stuff */
  public void setManualVoltageOnly(boolean manualVoltageOnly) {
    this.manualVoltageOnly = manualVoltageOnly;
  }

  public void setVoltage(double volts) {
    pid.setSetpoint(volts, ControlType.kVoltage);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public OptionalDouble getPositionOptional() {
    double val = getPosition();
    if(flex.getLastError() == REVLibError.kOk) {
      return OptionalDouble.of(val);
    } else {
      return OptionalDouble.empty();
    }
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }

  /* then this, after a delay */
  public void configure() {
    flex.setCANTimeout(250);
    config.smartCurrentLimit(DRIVE_CURRENT_LIMIT);
    config.idleMode(IdleMode.kBrake);
    config.inverted(IS_INVERTED[index]);
    config.voltageCompensation(12);
    config.encoder.positionConversionFactor(DRIVE_POS_FACTOR);
    config.encoder.velocityConversionFactor(DRIVE_VEL_FACTOR);
    config.closedLoop.p((DRIVE_PID[0] / 12.0) * DRIVE_VEL_FACTOR);
    config.closedLoop.i(0);
    config.closedLoop.d(0);

    config.closedLoop.feedForward.kS(DRIVE_FEEDFORWARD.getKs());
    config.closedLoop.feedForward.kV(DRIVE_FEEDFORWARD.getKv());
    config.closedLoop.feedForward.kA(DRIVE_FEEDFORWARD.getKa());

    config.signals.appliedOutputPeriodMs(20);
    config.signals.primaryEncoderPositionPeriodMs((int)(1000.0 / ODOMETRY_FREQUENCY));
    config.signals.primaryEncoderVelocityPeriodMs((int)(1000.0 / ODOMETRY_FREQUENCY));

    flex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    flex.setCANTimeout(0);
  }

  public void setBrakeCoast(boolean willBrake) {
    var updatedConfig = new SparkFlexConfig().idleMode(willBrake ? IdleMode.kBrake : IdleMode.kCoast);
    flex.configureAsync(updatedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public double getCurrent() {
    return flex.getOutputCurrent();
  }
}