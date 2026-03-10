package frc.robot.subsystems.spindexer;

import frc.robot.constants.SpindexerConstants;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Spindexer extends SubsystemBase {
    SparkFlex mainSpinner = new SparkFlex(SpindexerConstants.SPINNER_ID, MotorType.kBrushless);
    SparkFlexConfig mainSpinnerConfig = new SparkFlexConfig();

    SparkFlex feeder = new SparkFlex(SpindexerConstants.FEEDER_ID, MotorType.kBrushless);
    RelativeEncoder feederEncoder = feeder.getEncoder();
    SparkFlexConfig feederConfig = new SparkFlexConfig();
    SparkClosedLoopController feederController = feeder.getClosedLoopController();

    public Spindexer() {
        configure();
    }

    public void configure() {
        mainSpinnerConfig.smartCurrentLimit(SpindexerConstants.SPINNER_CURRENT_LIMIT);
        feederConfig.smartCurrentLimit(SpindexerConstants.FEEDER_CURRENT_LIMIT);
        mainSpinnerConfig.idleMode(IdleMode.kCoast);
        feederConfig.idleMode(IdleMode.kCoast);

        feederConfig.encoder.quadratureMeasurementPeriod(20);
        feederConfig.encoder.quadratureAverageDepth(4); 

        mainSpinnerConfig.encoder.quadratureMeasurementPeriod(20);
        mainSpinnerConfig.encoder.quadratureAverageDepth(4); 

        feederConfig.closedLoop.pid(SpindexerConstants.FEEDER_PID[0] / 12.0, SpindexerConstants.FEEDER_PID[1] / 12.0, SpindexerConstants.FEEDER_PID[2] / 12.0);
        feederConfig.closedLoop.feedForward.kS(SpindexerConstants.FEEDER_FEEDFORWARD.getKs());
        feederConfig.closedLoop.feedForward.kV(SpindexerConstants.FEEDER_FEEDFORWARD.getKv());
        feederConfig.closedLoop.feedForward.kA(SpindexerConstants.FEEDER_FEEDFORWARD.getKa());
               
        mainSpinner.configure(mainSpinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setVoltageMainSpinner(double voltage) {
        mainSpinner.setVoltage(voltage);
    }

    public void setVoltageFeeder(double volts) {
        feeder.setVoltage(volts);
    }
    
    public void setVoltageFeeder(Voltage volts) {
        setVoltageFeeder(volts.magnitude());
    }
    
    public void setFeederSpeed(double speed) {
        feederController.setSetpoint(speed, ControlType.kVelocity);
    }
    
    @AutoLogOutput 
    public double getMainSpinnerVoltage() {
        return mainSpinner.getAppliedOutput() * mainSpinner.getBusVoltage();
    }
    
    @AutoLogOutput
    public double getFeederVoltage() {
        return feeder.getAppliedOutput()*feeder.getBusVoltage();
    }

    @AutoLogOutput
    public double getFeederVelocity() {
        return feederEncoder.getVelocity();
    }

    @AutoLogOutput
    public double getFeederPosition() {
        return feederEncoder.getPosition();
    }

    @AutoLogOutput
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

   public SysIdRoutine getFeederSysidRoutine() {
    return new SysIdRoutine(new Config(Volts.of(.5).per(Second), Volts.of(7), null, (state)->{Logger.recordOutput("Spindexer/feederSysidTestState", state.toString());}),
        new Mechanism(this::setVoltageFeeder, null, this));
   }
    
}