package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.ConsoleSource.RoboRIO;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.SpindexerConstants;

public class Intake extends SubsystemBase {
    public SparkFlex deploy = new SparkFlex(DEPLOY_ID, MotorType.kBrushless);  // vortex
    public AbsoluteEncoder deployEncoder = deploy.getAbsoluteEncoder();
    public SparkClosedLoopController deployController = deploy.getClosedLoopController();
    public SparkFlexConfig deployConfig = new SparkFlexConfig();

    public SparkMax intake = new SparkMax(INTAKE_ID, MotorType.kBrushless);  // neo 2.0
    public SparkMaxConfig intakeConfig = new SparkMaxConfig();

    public Intake() {
        configure();
    }

    public void configure() {
        intake.setCANTimeout(250);
        deploy.setCANTimeout(250);

        deployConfig.smartCurrentLimit(IntakeConstants.DEPLOY_CURRENT_LIMIT);
        intakeConfig.smartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);
        deployConfig.idleMode(IdleMode.kBrake);
        intakeConfig.idleMode(IdleMode.kBrake);

        // deployConfig.

        // we prolly dont need ff
        deployConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);  // pid of absolute encoder is technically bad but if it doesn't work we'll find out
        deployConfig.closedLoop.p(DEPLOY_PID[0]);
        deployConfig.closedLoop.i(DEPLOY_PID[1]);
        deployConfig.closedLoop.d(DEPLOY_PID[2]);

        deploy.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        intake.setCANTimeout(0);
        deploy.setCANTimeout(0);
    }

    public void setIntakeVoltage(double volts) {
        intake.setVoltage(volts);
    }

    public void setDeployVoltage(double volts) {
        deployController.setSetpoint(volts, ControlType.kVoltage);
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
        return Rotation2d.fromRadians(deployEncoder.getPosition());
    }

    public void stopIntake() {
        intake.setVoltage(0);
    }

    @AutoLogOutput
    public double getIntakeVoltage() {
        return intake.get() * intake.getBusVoltage();
    }    
}
