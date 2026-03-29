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

import edu.wpi.first.math.MathUtil;
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
  SplineEncoder deploySplineEncoder = new SplineEncoder(DEPLOY_ENCODER_ID);
  RelativeEncoder deployRelativeEncoder = deploy.getEncoder();
  EncoderConfig deployRelativeEncoderConfig = new EncoderConfig();
  DetachedEncoderConfig deployAbsEncoderConfig = new DetachedEncoderConfig();
  SparkClosedLoopController deployController = deploy.getClosedLoopController();
  SparkFlexConfig deployConfig = new SparkFlexConfig();

  TrapezoidProfile profile = new TrapezoidProfile(DEPLOY_CONSTRAINTS);
  Rotation2d targetAngle;
  TrapezoidProfile.State profileSetpoint;
  @AutoLogOutput
  boolean voltageMode = false;
  @AutoLogOutput
  double holdVoltage;

  public Deploy() {
    configure();
    deployRelativeEncoder.setPosition(deploySplineEncoder.getAngle());
    targetAngle = getAngle();
    profileSetpoint = new State(getAngle().getRadians(), getIntegratedRelativeDeployVelocity());
  }

  void configure() {
    deployConfig.encoder.quadratureMeasurementPeriod(10);
    deployConfig.encoder.quadratureAverageDepth(2);
    deployConfig.closedLoop.feedbackSensor(FeedbackSensor.kDetachedAbsoluteEncoder, DEPLOY_ENCODER_ID);
    deployConfig.smartCurrentLimit(IntakeConstants.DEPLOY_CURRENT_LIMIT);
    deployConfig.idleMode(IdleMode.kBrake);
    deployConfig.disableVoltageCompensation();
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
    deploySplineEncoder.configure(deployAbsEncoderConfig, ResetMode.kResetSafeParameters);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      deployRelativeEncoder.setPosition(deploySplineEncoder.getAngle());
      profileSetpoint = new State(getAngle().getRadians(), getIntegratedRelativeDeployVelocity());
      targetAngle = getAngle();
    }

    if(voltageMode) {
      profileSetpoint = new State(getAngle().getRadians(), getIntegratedRelativeDeployVelocity());
      targetAngle = getAngle();
      voltageMode = false; // voltageMode should be continuously set to set a voltage, so we default to maintaining pid when no command is being called
    } else {
      profileSetpoint = profile.calculate(0.02, profileSetpoint, new State(targetAngle.getRadians(), holdVoltage));
      deployController.setSetpoint(profileSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      Logger.recordOutput("Deploy/currentDeploySetpoint", profileSetpoint);
    }

    holdVoltage = 0;
  }

  void setVoltage(double volts) {
    voltageMode = true;
    if (getAngle().getRadians() > DEPLOY_MAX_ANGLE.getRadians() && volts > 0) {
      deployController.setSetpoint(0, ControlType.kVoltage);
    } else if (getAngle().getRadians() < DEPLOY_MIN_ANGLE.getRadians() && volts < 0) {
      deployController.setSetpoint(0, ControlType.kVoltage);
    } else {
      deployController.setSetpoint(volts, ControlType.kVoltage);
    }
  }
  
  void setVoltage(Voltage volts) {
    setVoltage(volts.magnitude());
  }

  @AutoLogOutput
  public Rotation2d getAngle() {
    // return Rotation2d.fromRadians(deploySplineEncoder.getAngle());
    return Rotation2d.fromRadians(deployRelativeEncoder.getPosition());
  }

  @AutoLogOutput
  public Rotation2d getSplineEncoderRelativeDeployAngle() {
      return Rotation2d.fromRadians(deploySplineEncoder.getPosition());
  }

  @AutoLogOutput
  public Rotation2d getIntegratedRelativeDeployAngle() {
      return Rotation2d.fromRadians(deployRelativeEncoder.getPosition());
  }

  @AutoLogOutput
  public double getIntegratedRelativeDeployVelocity() {
      return deployRelativeEncoder.getVelocity();
  }

  @AutoLogOutput
  public double getSpineRelativeDeployVelocity() {
      return deploySplineEncoder.getVelocity();
  }

  public void setAngle(Rotation2d angle) {
    if (voltageMode) {
      profileSetpoint = new State(getAngle().getRadians(), getIntegratedRelativeDeployVelocity());
    }
    voltageMode = false;
    targetAngle = angle;
    Logger.recordOutput("Deploy/DeployLastGoalAngle", angle);
  }

  public boolean atAngle(Rotation2d angle) {
      return MathUtil.isNear(angle.getRadians(), getAngle().getRadians(), DEPLOY_ANGLE_TOLERANCE.getRadians());
  }

  public Command shimmy(Intake intake) {
    return Commands.repeatingSequence(
      this.setAngleCommand(IntakeConstants.DEPLOY_SHIMMY_HIGH_ANGLE).withTimeout(SHIMMY_TIMEOUT),
      this.setAngleCommand(IntakeConstants.DEPLOY_SHIMMY_LOW_ANGLE).withTimeout(SHIMMY_TIMEOUT)
    ).alongWith(intake.run(()->{intake.setIntakeVoltage(12);})).andThen(this.setAngleCommand(DEPLOY_MIN_ANGLE));
  }

  public Command holdDownCommand() {
    return this.run(()->{
      setAngle(DEPLOY_MIN_ANGLE);
      if (MathUtil.isNear(DEPLOY_MIN_ANGLE.getRadians(), getAngle().getRadians(), DEPLOY_ANGLE_TOLERANCE.getRadians())) {
        holdVoltage = DEPLOY_STALL_VOLTAGE;
      } else {
        holdVoltage = 0;  // this isn't necessary but wtv
      }
    });
  }

  @AutoLogOutput
  public boolean isAtGoal() {
    return deployController.isAtSetpoint() && MathUtil.isNear(profileSetpoint.position, targetAngle.getRadians(), 1e-3);
  }

  public Command setAngleCommand(Rotation2d angle) {
    return this.run(()->{
      setAngle(angle);
    });
  }

  public Command setVoltageCommand(double volts) {
    return this.run(()->{
      voltageMode = true;
      setVoltage(volts);
    });
  }

  public Command waitUntilAtGoal() {
    return this.idle().until(this::isAtGoal);
  }

  public boolean getForwardSysidLimit() {
    return getAngle().getRadians() > DEPLOY_MAX_SYSID_ANGLE.getRadians();
  }

  public boolean getBackwardSysidLimit() {
    return getAngle().getRadians() < DEPLOY_MIN_SYSID_ANGLE.getRadians();
  }

  @AutoLogOutput
  public double getDeploySetpoint() {
    return deployController.getSetpoint();
  }

  // @AutoLogOutput
  // public ControlType getDeployControlType() {
  //   return deployController.getControlType();  // this always logs duty cycle even when def in position mode?
  // }

  public SysIdRoutine getDeploySysid() {
    return new SysIdRoutine(
        new Config(Volts.of(.5).per(Second), Volts.of(1.5), null, (state) -> {
          Logger.recordOutput("Deploy/deploySysidTestState", state.toString());
        }),
        new Mechanism(this::setVoltage, null, this));
  }
}
