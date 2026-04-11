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
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Spindexer extends SubsystemBase {
    SparkFlex mainSpinner = new SparkFlex(SpindexerConstants.SPINNER_ID, MotorType.kBrushless);
    SparkFlexConfig mainSpinnerConfig = new SparkFlexConfig();

    SparkMax floober = new SparkMax(SpindexerConstants.FLOOBER_ID, MotorType.kBrushless);
    SparkMaxConfig flooberConfig = new SparkMaxConfig();

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

        feederConfig.encoder.positionConversionFactor(SpindexerConstants.FEEDER_POS_RATIO);
        feederConfig.encoder.velocityConversionFactor(SpindexerConstants.FEEDER_VEL_RATIO);

        mainSpinnerConfig.encoder.quadratureMeasurementPeriod(20);
        mainSpinnerConfig.encoder.quadratureAverageDepth(4); 

        feederConfig.closedLoop.pid(SpindexerConstants.FEEDER_PID[0] / 12.0, SpindexerConstants.FEEDER_PID[1] / 12.0, SpindexerConstants.FEEDER_PID[2] / 12.0);
        // feederConfig.closedLoop.feedForward.kS(SpindexerConstants.FEEDER_FEEDFORWARD.getKs());
        // feederConfig.closedLoop.feedForward.kV(SpindexerConstants.FEEDER_FEEDFORWARD.getKv());
        // feederConfig.closedLoop.feedForward.kA(SpindexerConstants.FEEDER_FEEDFORWARD.getKa());

        flooberConfig.idleMode(IdleMode.kCoast);
        flooberConfig.inverted(true);
        flooberConfig.smartCurrentLimit(SpindexerConstants.FLOOBER_CURRENT_LIMIT);

        floober.configure(flooberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);       
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

    public void setVoltageFloober(double volts) {
        floober.setVoltage(volts);
    }
    
    public void setFeederSpeed(double velocity) {
        double ff = SpindexerConstants.FEEDER_FEEDFORWARD.calculate(velocity);
        Logger.recordOutput("Spindexer/Feederff", ff);
        feederController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff);
    }
    
    @AutoLogOutput 
    public double getMainSpinnerVoltage() {
        return mainSpinner.getAppliedOutput() * mainSpinner.getBusVoltage();
    }

    @AutoLogOutput
    public double getFlooberVoltage() {
        return floober.getAppliedOutput() * floober.getBusVoltage();
    }

    @AutoLogOutput
    public double getFeederVelocity() {
        return feederEncoder.getVelocity();
    }

    @AutoLogOutput
    public boolean feederAtSpeed() {
        return MathUtil.isNear(SpindexerConstants.FEED_VELOCITY, getFeederVelocity(), SpindexerConstants.FEEDER_SPEED_TOLERANCE);
    }

    @AutoLogOutput
    public double getFeederSetpoint() {
        return feederController.getSetpoint();
    }

    public void stop() {
        feeder.stopMotor(); 
        mainSpinner.stopMotor();
    }

    public Command runSpindexer(double volts) {
        return this.run(() -> {
            setVoltageMainSpinner(-volts);
            setVoltageFeeder(volts);
        }).finallyDo(() -> {
            setVoltageMainSpinner(0);
            setVoltageFeeder(0);
        });
    }

    public Command runWhenReady() {
        return this.run(() -> {
            setVoltageFeeder(11);
        }).until(this::feederAtSpeed).andThen(this.run(()->{
            setVoltageMainSpinner(-12);
            setVoltageFeeder(11);
        })).finallyDo(() -> {
            setVoltageMainSpinner(0);
            setVoltageFeeder(0);
        });
    }

   public SysIdRoutine getFeederSysidRoutine() {
    return new SysIdRoutine(new Config(Volts.of(.5).per(Second), Volts.of(7), null, (state)->{Logger.recordOutput("Spindexer/feederSysidTestState", state.toString());}),
        new Mechanism(this::setVoltageFeeder, null, this));
   }
    
}