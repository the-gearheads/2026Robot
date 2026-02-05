package frc.robot.subsystems.swerve.motors;

import static frc.robot.constants.SwerveConstants.*;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.geometry.Rotation2d;

public class SteerMotor {

  SparkMax max;
  SparkAbsoluteEncoder encoder;
  SparkClosedLoopController pid;
  SparkMaxConfig config = new SparkMaxConfig();

  int index;
  Rotation2d offset;
  String modulePath;

  double targetAngle;

  // mifdsa checkadfs
  boolean manualVoltageOnly = false;

  public SteerMotor(int id, int index, Rotation2d offset, String modulePath) {
    max = new SparkMax(id, MotorType.kBrushless);
    encoder = max.getAbsoluteEncoder();
    pid = max.getClosedLoopController();
    this.index = index;
    this.offset = offset;
    this.modulePath = modulePath;
  }
  
  public void noOffsetSetAngle(Rotation2d angle) {
    if(manualVoltageOnly) return;
    pid.setSetpoint(angle.getRadians(), ControlType.kPosition);
  }

  public void setAngle(Rotation2d angle) {
    targetAngle = angle.getRadians();
    noOffsetSetAngle(angle.plus(offset));
  }

  public Rotation2d noOffsetGetAngle() {
    return Rotation2d.fromRadians(encoder.getPosition());
  }

  public Rotation2d getAngle() {
    return noOffsetGetAngle().minus(offset);
  }

  public double getAngleRadians() {
    return encoder.getPosition() - offset.getRadians();
  }

  public OptionalDouble getAngleRadiansOptional() {
    double val = getAngleRadians();
    if(max.getLastError() == REVLibError.kOk) {
      return OptionalDouble.of(val);
    } else {
      return OptionalDouble.empty();
    }
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

  public void log() {
    Logger.recordOutput(modulePath + "/steerVolts", max.getAppliedOutput() * max.getBusVoltage());
    Logger.recordOutput(modulePath + "/steerAngle", getAngle().getRadians());
    Logger.recordOutput(modulePath + "/steerVel", encoder.getVelocity());
    Logger.recordOutput(modulePath + "/targetAngle", targetAngle);
    Logger.recordOutput(modulePath + "/manualVoltageOnly", manualVoltageOnly);
  }
  
  public void configure() {
    max.setCANTimeout(250); 
    config.smartCurrentLimit(STEER_CURRENT_LIMIT);
    config.idleMode(IdleMode.kBrake);

    config.absoluteEncoder.positionConversionFactor(STEER_POS_FACTOR);
    config.absoluteEncoder.velocityConversionFactor(STEER_VEL_FACTOR);
    config.absoluteEncoder.inverted(true);

    config.closedLoop.positionWrappingEnabled(true);
    config.closedLoop.positionWrappingMinInput(0);
    config.closedLoop.positionWrappingMaxInput(Math.PI * 2);

    config.closedLoop.pid(STEER_PIDF[0], STEER_PIDF[1], STEER_PIDF[2]);
    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);


    // I currently do not know whether revlib takes the minumum of all signals in a status frame including or excluding defaults.
    config.signals.appliedOutputPeriodMs(20);
    config.signals.primaryEncoderPositionAlwaysOn(false);
    config.signals.primaryEncoderVelocityPeriodMs(40);

    config.signals.absoluteEncoderPositionAlwaysOn(true);
    config.signals.absoluteEncoderPositionPeriodMs((int)(1000.0 / ODOMETRY_FREQUENCY));
    config.signals.absoluteEncoderVelocityAlwaysOn(true);
    config.signals.absoluteEncoderVelocityPeriodMs(20);

    max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    max.setCANTimeout(0);
  }

  public void setBrakeCoast(boolean willBrake) {
    var updatedConfig = new SparkMaxConfig().idleMode(willBrake ? IdleMode.kBrake : IdleMode.kCoast);
    max.configureAsync(updatedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public double getCurrent() {
    return max.getOutputCurrent();
  }
}