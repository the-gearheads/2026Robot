package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;


public class Shooter extends SubsystemBase{
    
    SparkFlex mainFly = new SparkFlex(MAIN_FLY_ID, MotorType.kBrushless);
    SparkClosedLoopController flywheelController = mainFly.getClosedLoopController();
    SparkFlexConfig mainFlyConfig = new SparkFlexConfig();
    RelativeEncoder flywheelEncoder = mainFly.getEncoder(); 
    
    SparkFlex followerFly = new SparkFlex(FOLLOWER_FLY_ID, MotorType.kBrushless);
    SparkFlexConfig followerFlyConfig = new SparkFlexConfig();

    SparkFlex kicker = new SparkFlex(KICKER_ID, MotorType.kBrushless);
    SparkClosedLoopController kickerController = kicker.getClosedLoopController();
    SparkFlexConfig kickerConfig = new SparkFlexConfig();
    RelativeEncoder kickerEncoder = kicker.getEncoder();

   public Shooter() {
    configure();
   }
   
   @Override
   public void periodic(){
    
   }

   public void configure() {
    mainFly.setCANTimeout(250);
    followerFly.setCANTimeout(250);
    kicker.setCANTimeout(250);
    
    mainFlyConfig.smartCurrentLimit(65);
    mainFlyConfig.inverted(false);
    mainFlyConfig.idleMode(IdleMode.kBrake);
    mainFlyConfig.voltageCompensation(12);
    mainFlyConfig.closedLoop.pid(FLYWHEEL_PID[0], FLYWHEEL_PID[1], FLYWHEEL_PID[2]);
    mainFlyConfig.closedLoop.feedForward.kS(FLYWHEEL_FEEDFORWARD.getKs());
    mainFlyConfig.closedLoop.feedForward.kV(FLYWHEEL_FEEDFORWARD.getKv());
    mainFlyConfig.closedLoop.feedForward.kA(FLYWHEEL_FEEDFORWARD.getKa());

    mainFlyConfig.encoder.positionConversionFactor(FLYWHEEL_POS_FACTOR);
    mainFlyConfig.encoder.velocityConversionFactor(FLYWHEEL_POS_FACTOR);
    
    followerFlyConfig.smartCurrentLimit(65);
    followerFlyConfig.idleMode(IdleMode.kBrake);
    followerFlyConfig.follow(mainFly, true);
    followerFlyConfig.voltageCompensation(12);

    kickerConfig.smartCurrentLimit(65);
    kickerConfig.inverted(false);
    kickerConfig.idleMode(IdleMode.kBrake);
    kickerConfig.voltageCompensation(12);
    kickerConfig.closedLoop.pid(KICKER_PID[0], KICKER_PID[1], KICKER_PID[2]);
    kickerConfig.closedLoop.feedForward.kS(KICKER_FEEDFORWARD.getKs());
    kickerConfig.closedLoop.feedForward.kV(KICKER_FEEDFORWARD.getKv());
    kickerConfig.closedLoop.feedForward.kA(KICKER_FEEDFORWARD.getKa());

    kickerConfig.encoder.positionConversionFactor(KICKER_POS_FACTOR);
    kickerConfig.encoder.velocityConversionFactor(KICKER_VEL_FACTOR);

    kicker.setCANTimeout(0);
    followerFly.setCANTimeout(0);
    mainFly.setCANTimeout(0);

    mainFly.configure(mainFlyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kicker.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerFly.configure(followerFlyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }

   public void setFlywheelVoltage(double volts) {
    flywheelController.setSetpoint(volts, ControlType.kVoltage);
   }
   
   public void setFlywheelVoltage(Voltage volts) {
    setFlywheelVoltage(volts.magnitude());
   }
   
   public void setKickerVoltage(double volts){
    kickerController.setSetpoint(volts, ControlType.kVoltage);
   }

   public void setKickerVoltage(Voltage volts) {
    setKickerVoltage(volts.magnitude());
   }

   @AutoLogOutput
   public ControlType getKickerControlType() {
    return kickerController.getControlType();
   }
   
   @AutoLogOutput
   public ControlType getFlywheelControlType(){
    return flywheelController.getControlType();
   }

   public void setFlywheelVelocity(double velocity) {
    flywheelController.setSetpoint(velocity, ControlType.kVelocity);
   }

   public void setKickerVelocity(double velocity){
    kickerController.setSetpoint(velocity, ControlType.kVelocity);
   }

   @AutoLogOutput
   public double getKickerVelocity() {
    return kickerEncoder.getVelocity();
   }

   @AutoLogOutput
   public double getKickerSetpoint() {
    return kickerController.getSetpoint();
   }

   @AutoLogOutput
   public double getFlywheelVelocity() {
    return flywheelEncoder.getVelocity();
   }

   @AutoLogOutput
   public double getFlywheelSetpoint() {
    return flywheelController.getSetpoint();
   }

   @AutoLogOutput
   public double getFlywheelCurrent() {
    return mainFly.getOutputCurrent();
   }

   @AutoLogOutput
   public double getKickerCurrent() {
    return kicker.getOutputCurrent();
   }

   public Command runShooter(double volts){
    return this.run(() -> {
        setKickerVoltage(volts);
        setFlywheelVoltage(volts);
    });
   }

   public SysIdRoutine getMainFlySysidRoutine() {
    return new SysIdRoutine(new Config(Volts.of(.5).per(Second), Volts.of(7), null, (state)->{Logger.recordOutput("Shooter/mainFlySysidTestState", state.toString());}),
        new Mechanism(this::setFlywheelVoltage, null, this));
   }

   public SysIdRoutine getKickerSysidRoutine() {
    return new SysIdRoutine(new Config(Volts.of(.5).per(Second), Volts.of(7), null, (state)->{Logger.recordOutput("Shooter/kickerSysidTestState", state.toString());}),
        new Mechanism(this::setKickerVoltage, null, this));
   }
   
}
