package frc.robot.subsystems.spindexer;

import frc.robot.constants.SpindexerConstants;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
    SparkFlex mainSpinner = new SparkFlex(SpindexerConstants.SPINNER_ID, MotorType.kBrushless);
    SparkFlex feeder = new SparkFlex(SpindexerConstants.FEEDER_ID, MotorType.kBrushless);
    SparkFlexConfig mainSpinnerConfig = new SparkFlexConfig();
    SparkFlexConfig feederConfig = new SparkFlexConfig();
    SparkClosedLoopController feederController = feeder.getClosedLoopController();

    public Spindexer() {
        configure();
    }

    public void configure()
    {
        mainSpinner.setCANTimeout(250);
        feeder.setCANTimeout(250);

        mainSpinnerConfig.smartCurrentLimit(SpindexerConstants.SPINNER_CURRENT_LIMIT);
        feederConfig.smartCurrentLimit(SpindexerConstants.FEEDER_CURRENT_LIMIT);
        mainSpinnerConfig.idleMode(IdleMode.kBrake);
        feederConfig.idleMode(IdleMode.kBrake);

        mainSpinner.configure(mainSpinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

       feederConfig.closedLoop.pid(SpindexerConstants.FEEDER_PID[0], SpindexerConstants.FEEDER_PID[1], SpindexerConstants.FEEDER_PID[2]);
       feederConfig.closedLoop.feedForward.kS(SpindexerConstants.FEEDER_FEEDFORWARD.getKs());
       feederConfig.closedLoop.feedForward.kV(SpindexerConstants.FEEDER_FEEDFORWARD.getKv());
       feederConfig.closedLoop.feedForward.kA(SpindexerConstants.FEEDER_FEEDFORWARD.getKa());
        
        mainSpinner.setCANTimeout(0);
        feeder.setCANTimeout(0);
    }

    public void setVoltageMainSpinner(double voltage) {
        mainSpinner.setVoltage(voltage);
    }

    public void setVoltageFeeder(double voltage) {
        feederController.setSetpoint(voltage, ControlType.kVoltage);
    }
    
    @AutoLogOutput 
    public double getMainSpinnerVoltage() {
        return mainSpinner.getAppliedOutput() * mainSpinner.getBusVoltage();
    }
    
    @AutoLogOutput
    public double getFeederVoltage() {
        return feeder.getAppliedOutput()*feeder.getBusVoltage();
    }

    public double getFeederSetpoint() {
        return feederController.getSetpoint();
    }

    public void stop() {
        feeder.stopMotor(); 
        mainSpinner.stopMotor();
    }

    public Command runSpindexer(double volts){
    return this.run(() -> {
        setVoltageMainSpinner(volts);
        setVoltageFeeder(volts);
    }).finallyDo(() -> {
        setVoltageMainSpinner(0);
        setVoltageFeeder(0);
    });
   }
    
}