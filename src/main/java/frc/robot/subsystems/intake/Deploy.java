package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.IntakeConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.encoder.SplineEncoder;
import com.revrobotics.encoder.config.DetachedEncoderConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.constants.IntakeConstants;

public class Deploy extends SubsystemBase {
  SparkFlex deploy = new SparkFlex(DEPLOY_ID, MotorType.kBrushless); // vortex
  SplineEncoder deployEncoder = new SplineEncoder(DEPLOY_ENCODER_ID);
  RelativeEncoder deployRelativeEncoder = deploy.getEncoder();
  EncoderConfig deployRelativeEncoderConfig = new EncoderConfig();
  DetachedEncoderConfig deployAbsEncoderConfig = new DetachedEncoderConfig();
  SparkClosedLoopController deployController = deploy.getClosedLoopController();
  SparkFlexConfig deployConfig = new SparkFlexConfig();

  TrapezoidProfile profile = new TrapezoidProfile(DEPLOY_CONSTRAINTS);
  Rotation2d targetAngle;
  TrapezoidProfile.State profileSetpoint;
  boolean isManualMode = false;

  public Deploy() {
    deployRelativeEncoder.setPosition(deployEncoder.getAngle());
    targetAngle = Rotation2d.fromRadians(deployEncoder.getAngle());
    profileSetpoint = new State(getRelativeDeployAngle().getRadians(), getRelativeDeployVelocity());

  }

  void configure() {
    deployConfig.encoder.quadratureMeasurementPeriod(10);
    deployConfig.encoder.quadratureAverageDepth(2);
    deployConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    deployConfig.smartCurrentLimit(IntakeConstants.DEPLOY_CURRENT_LIMIT);
    deployConfig.idleMode(IdleMode.kBrake);
    deployConfig.inverted(true);

    deployAbsEncoderConfig.positionConversionFactor(DEPLOY_ABS_ENC_POS_FACTOR);
    deployAbsEncoderConfig.angleConversionFactor(DEPLOY_ABS_ENC_POS_FACTOR);
    deployAbsEncoderConfig.velocityConversionFactor(DEPLOY_ABS_ENC_VEL_FACTOR);
    deployAbsEncoderConfig.inverted(true);
    deployAbsEncoderConfig.dutyCycleZeroCentered(true);
    deployAbsEncoderConfig.dutyCycleOffset(DEPLOY_ABS_ENC_OFFSET);

    deployConfig.encoder.positionConversionFactor(DEPLOY_POS_FACTOR);
    deployConfig.encoder.velocityConversionFactor(DEPLOY_VEL_FACTOR);

    // we prolly dont need ff
    deployConfig.closedLoop.p(DEPLOY_PID[0]);
    deployConfig.closedLoop.i(DEPLOY_PID[1]);
    deployConfig.closedLoop.d(DEPLOY_PID[2]);
    deploy.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    deployEncoder.configure(deployAbsEncoderConfig, ResetMode.kResetSafeParameters);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      deployRelativeEncoder.setPosition(deployEncoder.getAngle());
    }

    if (!isManualMode) {
      profileSetpoint = profile.calculate(0.02, profileSetpoint, new State(targetAngle.getRadians(), 0));
      deployController.setSetpoint(profileSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      Logger.recordOutput("Intake/currentDeploySetpoint", profileSetpoint);
    } else {
      profileSetpoint = new State(getRelativeDeployAngle().getRadians(), getRelativeDeployVelocity());
    }

  }

  public void setDeployVoltage(double volts) {
    isManualMode = true;
    if (getAngle().getRadians() > DEPLOY_MAX_ANGLE.getRadians() && volts > 0) {
      deployController.setSetpoint(0, ControlType.kVoltage);
    } else if (getAngle().getRadians() < DEPLOY_MIN_ANGLE.getRadians() && volts < 0) {
      deployController.setSetpoint(0, ControlType.kVoltage);
    } else {
      deployController.setSetpoint(volts, ControlType.kVoltage);
    }
  }

  @AutoLogOutput
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(deployEncoder.getAngle());
  }

  @AutoLogOutput
  public Rotation2d getRelativeDeployAngle() {
    return Rotation2d.fromRadians(deployEncoder.getPosition());
  }

  @AutoLogOutput
  public double getRelativeDeployVelocity() {
    return deployEncoder.getVelocity();
  }

  public void setAngle(Rotation2d angle) {
    if (isManualMode) {
      profileSetpoint = new State(getRelativeDeployAngle().getRadians(), getRelativeDeployVelocity());
    }
    isManualMode = false;
    targetAngle = angle;
    Logger.recordOutput("Intake/DeployLastGoalAngle", angle);
    deployController.setSetpoint(angle.getRadians(), ControlType.kPosition);
  }

  public Command shimmy(Intake intake) {
    return Commands.run(() -> {

      if (Math.abs(getAngle().getRadians() - DEPLOY_MIN_ANGLE.getRadians()) < DEPLOY_SHIMMY_TOLERANCE) {
        setAngle(DEPLOY_SHIMMY_ANGLE);
      }

      if (Math.abs(getAngle().getRadians() - DEPLOY_SHIMMY_ANGLE.getRadians()) < DEPLOY_SHIMMY_TOLERANCE) {
        setAngle(DEPLOY_MIN_ANGLE);
      }

      intake.setIntakeVelocity(INTAKE_VELOCITY);
    }, this, intake);
  }

  public boolean getForwardSysidLimit() {
    return getAngle().getRadians() > DEPLOY_MAX_SYSID_ANGLE.getRadians();
  }

  public boolean getBackwardSysidLimit() {
    return getAngle().getRadians() < DEPLOY_MIN_SYSID_ANGLE.getRadians();
  }

  public void setDeployVoltage(Voltage volts) {
    deploy.setVoltage(volts);
  }

  @AutoLogOutput
  public double getDeploySetpoint() {
    return deployController.getSetpoint();
  }

  @AutoLogOutput
  public ControlType getDeployControlType() {
    return deployController.getControlType();
  }

  public SysIdRoutine getDeploySysid() {
    return new SysIdRoutine(
        new Config(Volts.of(.5).per(Second), Volts.of(1.5), null, (state) -> {
          Logger.recordOutput("Intake/deploySysidTestState", state.toString());
        }),
        new Mechanism(this::setDeployVoltage, null, this));
  }

}
