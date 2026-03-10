package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.IntakeConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.encoder.SplineEncoder;
import com.revrobotics.encoder.config.DetachedEncoderConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    public SparkFlex deploy = new SparkFlex(DEPLOY_ID, MotorType.kBrushless);  // vortex
    public SplineEncoder deployEncoder = new SplineEncoder(DEPLOY_ENCODER_ID);
    public DetachedEncoderConfig deployEncoderConfig = new DetachedEncoderConfig();
    public SparkClosedLoopController deployController = deploy.getClosedLoopController();
    public SparkFlexConfig deployConfig = new SparkFlexConfig();

    public SparkMax intake = new SparkMax(INTAKE_ID, MotorType.kBrushless);  // neo 2.0
    public SparkMaxConfig intakeConfig = new SparkMaxConfig();

    public Intake() {
        configure();
        deployEncoder.setPosition(deployEncoder.getAngle());
    }

    public void configure() {
        intake.setCANTimeout(10);
        deploy.setCANTimeout(10);

        deployConfig.encoder.quadratureMeasurementPeriod(10);
        deployConfig.encoder.quadratureAverageDepth(2); 

        intakeConfig.encoder.quadratureMeasurementPeriod(64);
        intakeConfig.encoder.quadratureAverageDepth(16); 

        deployConfig.smartCurrentLimit(IntakeConstants.DEPLOY_CURRENT_LIMIT);
        intakeConfig.smartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);
        deployConfig.idleMode(IdleMode.kBrake);
        intakeConfig.idleMode(IdleMode.kCoast);

        intakeConfig.inverted(true);
        deployConfig.inverted(true);

        deployEncoderConfig.positionConversionFactor(DEPLOY_POS_FACTOR);
        deployEncoderConfig.angleConversionFactor(DEPLOY_POS_FACTOR);
        deployEncoderConfig.velocityConversionFactor(DEPLOY_VEL_FACTOR);
        deployEncoderConfig.inverted(true);
        deployEncoderConfig.dutyCycleZeroCentered(true);
        deployEncoderConfig.dutyCycleOffset(DEPLOY_OFFSET);

        deployConfig.closedLoop.feedbackSensor(FeedbackSensor.kDetachedAbsoluteEncoder, DEPLOY_ENCODER_ID);
        // we prolly dont need ff
        deployConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);  // pid off of absolute encoder is technically bad but if it doesn't work we'll find out
        deployConfig.closedLoop.p(DEPLOY_PID[0]);
        deployConfig.closedLoop.i(DEPLOY_PID[1]);
        deployConfig.closedLoop.d(DEPLOY_PID[2]);

        deploy.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        deployEncoder.configure(deployEncoderConfig, ResetMode.kResetSafeParameters);
                
        intake.setCANTimeout(0);
        deploy.setCANTimeout(0);
    }

    public void setIntakeVoltage(Voltage volts) {
        intake.setVoltage(volts);
    }

       public void setIntakeVoltage(double volts) {
        deployController.setSetpoint(volts, ControlType.kVoltage);
    }

    public void setDeployVoltage(double volts) {
        deployController.setSetpoint(volts, ControlType.kVoltage);
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

    public void setAngle(Rotation2d angle) {
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

    public Command shimmy() {
        return this.run(() -> {

            if (Math.abs(getAngle().getRadians() - DEPLOY_MIN_ANGLE.getRadians()) < DEPLOY_SHIMMY_TOLERANCE) {
                setAngle(DEPLOY_SHIMMY_ANGLE);
            }

            if (Math.abs(getAngle().getRadians() - DEPLOY_SHIMMY_ANGLE.getRadians()) < DEPLOY_SHIMMY_TOLERANCE) {
                setAngle(DEPLOY_MIN_ANGLE);
            }

            setIntakeVoltage(12);
        }).finallyDo(this::stopIntake);
    }

    public boolean getForwardSysidLimit() {
        return getAngle().getRadians() > DEPLOY_MAX_SYSID_ANGLE.getRadians();
    }

    public boolean getBackwardSysidLimit() {
        return getAngle().getRadians() < DEPLOY_MIN_SYSID_ANGLE.getRadians();
    }

    public SysIdRoutine getDeploySysid() {
        return new SysIdRoutine(
            new Config(Volts.of(.5).per(Second), Volts.of(2), null, (state)->{Logger.recordOutput("Intake/deploySysidTestState", state.toString());}),
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
