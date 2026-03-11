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
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    SparkFlex deploy = new SparkFlex(DEPLOY_ID, MotorType.kBrushless);  // vortex
    SplineEncoder deployEncoder = new SplineEncoder(DEPLOY_ENCODER_ID);
    RelativeEncoder deployRelativeEncoder = deploy.getEncoder();
    EncoderConfig deployRelativeEncoderConfig = new EncoderConfig();
    DetachedEncoderConfig deployAbsEncoderConfig = new DetachedEncoderConfig();
    SparkClosedLoopController deployController = deploy.getClosedLoopController();
    SparkFlexConfig deployConfig = new SparkFlexConfig();

    SparkMax intake = new SparkMax(INTAKE_ID, MotorType.kBrushless);  // neo 2.0
    RelativeEncoder intakEncoder = intake.getEncoder();
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    SparkClosedLoopController intakeController = intake.getClosedLoopController();

    TrapezoidProfile profile = new TrapezoidProfile(DEPLOY_CONSTRAINTS);
    Rotation2d targetAngle;
    TrapezoidProfile.State profileSetpoint;
    TrapezoidProfile.State lastSetpoint;
    
    @AutoLogOutput
    boolean isManualMode = false;
    public Intake() {
        configure();
        deployRelativeEncoder.setPosition(deployEncoder.getAngle());

        targetAngle = getAngle();
        profileSetpoint = new State(getAngle().getRadians(), getRelativeDeployVelocity());
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            deployRelativeEncoder.setPosition(deployEncoder.getAngle());
            targetAngle = getAngle();
            profileSetpoint = new State(getAngle().getRadians(), getRelativeDeployVelocity());
        }

        if (!isManualMode) {
            profileSetpoint = profile.calculate(0.02, profileSetpoint, new State(targetAngle.getRadians(), 0));
            deployController.setSetpoint(profileSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
            Logger.recordOutput("Intake/currentDeploySetpoint", profileSetpoint);
        } else {
            profileSetpoint = new State(getRelativeDeployAngle().getRadians(), getRelativeDeployVelocity());
            // targetAngle = getRelativeDeployAngle();
        }

    }

    public void configure() {
        deployConfig.encoder.quadratureMeasurementPeriod(10);
        deployConfig.encoder.quadratureAverageDepth(2); 

        intakeConfig.encoder.quadratureMeasurementPeriod(10);
        intakeConfig.encoder.quadratureAverageDepth(2); 

        deployConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        deployConfig.closedLoop.pid(DEPLOY_PID[0], DEPLOY_PID[1], DEPLOY_PID[2]);

        deployConfig.smartCurrentLimit(IntakeConstants.DEPLOY_CURRENT_LIMIT);
        intakeConfig.smartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);
        deployConfig.idleMode(IdleMode.kBrake);
        intakeConfig.idleMode(IdleMode.kCoast);

        intakeConfig.inverted(true);
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

        intakeConfig.closedLoop.feedForward.kA(INTAKE_FEEDFORWARD.getKa());
        intakeConfig.closedLoop.feedForward.kV(INTAKE_FEEDFORWARD.getKv());
        intakeConfig.closedLoop.feedForward.kS(INTAKE_FEEDFORWARD.getKs());

        intakeConfig.closedLoop.pid(INTAKE_PID[0]/12.0, INTAKE_PID[1]/12.0, INTAKE_PID[2]/12.0);

        deploy.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        deployEncoder.configure(deployAbsEncoderConfig, ResetMode.kResetSafeParameters);
                
    }

    public void setIntakeVoltage(Voltage volts) {
        intake.setVoltage(volts);
    }

    public void setIntakeVoltage(double volts) {
        intakeController.setSetpoint(volts, ControlType.kVoltage);
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

    public void setDeployVoltage(Voltage volts) {
        deploy.setVoltage(volts);
    }

    @AutoLogOutput
    public double getDeployControllerSetpoint() {
        return deployController.getSetpoint();
    }

    @AutoLogOutput
    public ControlType getDeployControlType() {
        return deployController.getControlType();
    }

    public void setAngle(Rotation2d angle) {
        if (isManualMode) {
            profileSetpoint = new State(getRelativeDeployAngle().getRadians(), getRelativeDeployVelocity());
            isManualMode = false;
        }
        targetAngle = angle;
        Logger.recordOutput("Intake/DeployLastGoalAngle", angle);
        deployController.setSetpoint(angle.getRadians(), ControlType.kPosition);
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

    public void stopIntake() {
        intake.setVoltage(0);
    }

    @AutoLogOutput
    public double getIntakeVoltage() {
        return intake.get() * intake.getBusVoltage();
    }    

    public double getIntakeVelocity() {
        return intakEncoder.getVelocity();
    }

    public void setIntakeVelocity(double velocity) {
        intakeController.setSetpoint(velocity, ControlType.kVelocity);
    } 

    public boolean atAngle(Rotation2d angle) {
        return MathUtil.isNear(angle.getRadians(), getRelativeDeployAngle().getRadians(), DEPLOY_ANGLE_TOLERANCE.getRadians());
    }

    public void goToShimmyAngle() {
        if (MathUtil.isNear(DEPLOY_SHIMMY_ANGLE.getRadians(), getAngle().getRadians(), DEPLOY_SHIMMY_TOLERANCE.getRadians())) {
            setAngle(DEPLOY_MIN_ANGLE);
        } else if (MathUtil.isNear(DEPLOY_MIN_ANGLE.getRadians(), getAngle().getRadians(), DEPLOY_SHIMMY_TOLERANCE.getRadians())){
            setAngle(DEPLOY_SHIMMY_ANGLE);
        }
    }

    public Command shimmy() {
        return this.runOnce(()->{setAngle(DEPLOY_SHIMMY_ANGLE);}).andThen(this.run(()->{
            this.setIntakeVoltage(12);
            this.goToShimmyAngle();
        }));
    }

    public boolean getForwardSysidLimit() {
        return getAngle().getRadians() > DEPLOY_MAX_SYSID_ANGLE.getRadians();
    }

    public boolean getBackwardSysidLimit() {
        return getAngle().getRadians() < DEPLOY_MIN_SYSID_ANGLE.getRadians();
    }

    public SysIdRoutine getDeploySysid() {
        return new SysIdRoutine(
            new Config(Volts.of(.5).per(Second), Volts.of(1.5), null, (state)->{Logger.recordOutput("Intake/deploySysidTestState", state.toString());}),
            new Mechanism(this::setDeployVoltage, null, this)
        );
    }

    public SysIdRoutine getIntakeSysid() {
        return new SysIdRoutine(
            new Config(Volts.of(.5).per(Second), Volts.of(2), null, (state)->{Logger.recordOutput("Intake/intakeSysidTestState", state.toString());}),
            new Mechanism(this::setIntakeVoltage, null, this)
        );
    }
}
