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
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    RelativeEncoder mainFlyEncoder = mainFly.getEncoder(); 
    
    SparkFlex followerFly = new SparkFlex(FOLLOWER_FLY_ID, MotorType.kBrushless);
    SparkFlexConfig followerFlyConfig = new SparkFlexConfig();

    SparkFlex topFly = new SparkFlex(TOP_ROLLER_ID, MotorType.kBrushless);
    SparkClosedLoopController topFlyController = topFly.getClosedLoopController();
    SparkFlexConfig topFlyConfig = new SparkFlexConfig();
    RelativeEncoder topFlyEncoder = topFly.getEncoder();

   public Shooter() {
    configure();

   }
   
   @Override
   public void periodic(){
    
   }

   public void configure() {
    mainFly.setCANTimeout(250);
    followerFly.setCANTimeout(250);
    topFly.setCANTimeout(250);
    
    mainFlyConfig.smartCurrentLimit(65);
    mainFlyConfig.inverted(false);
    mainFlyConfig.idleMode(IdleMode.kBrake);
    mainFlyConfig.voltageCompensation(12);
    mainFlyConfig.closedLoop.pid(FLYWHEEL_PID[0], FLYWHEEL_PID[1], FLYWHEEL_PID[2]);
    mainFlyConfig.closedLoop.feedForward.kS(FLYWHEEL_FEEDFORWARD.getKs());
    mainFlyConfig.closedLoop.feedForward.kV(FLYWHEEL_FEEDFORWARD.getKv());
    mainFlyConfig.closedLoop.feedForward.kA(FLYWHEEL_FEEDFORWARD.getKa());

    followerFlyConfig.smartCurrentLimit(65);
    followerFlyConfig.idleMode(IdleMode.kBrake);
    followerFlyConfig.follow(mainFly, true);
    followerFlyConfig.voltageCompensation(12);

    topFlyConfig.smartCurrentLimit(65);
    topFlyConfig.inverted(false);
    topFlyConfig.idleMode(IdleMode.kBrake);
    topFlyConfig.voltageCompensation(12);
    topFlyConfig.closedLoop.pid(TOPFLY_PID[0], TOPFLY_PID[1], TOPFLY_PID[2]);
    topFlyConfig.closedLoop.feedForward.kS(TOPFLY_FEEDFORWARD.getKs());
    topFlyConfig.closedLoop.feedForward.kV(TOPFLY_FEEDFORWARD.getKv());
    topFlyConfig.closedLoop.feedForward.kA(TOPFLY_FEEDFORWARD.getKa());

    topFly.setCANTimeout(0);
    followerFly.setCANTimeout(0);
    mainFly.setCANTimeout(0);

    mainFly.configure(mainFlyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topFly.configure(topFlyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerFly.configure(followerFlyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }

   public void setFlywheelVoltage(double volts) {
    flywheelController.setSetpoint(volts, ControlType.kVoltage);
   }
   
   public void setFlywheelVoltage(Voltage volts) {
    setFlywheelVoltage(volts.magnitude());
   }
   
   public void setTopFlyVoltage(double volts){
    topFlyController.setSetpoint(volts, ControlType.kVoltage);
   }

   public void setTopFlyVoltage(Voltage volts) {
    setTopFlyVoltage(volts.magnitude());
   }

   @AutoLogOutput
   public ControlType getTopFlyControlType() {
    return topFlyController.getControlType();
   }
   
   @AutoLogOutput
   public ControlType getFlywheelControlType(){
    return flywheelController.getControlType();
   }

   public void setFlywheelVelocity(double velocity) {
    flywheelController.setSetpoint(velocity, ControlType.kVelocity);
   }

   public void setTopFlyVelocity(double velocity){
    topFlyController.setSetpoint(velocity, ControlType.kVelocity);
   }

   @AutoLogOutput
   public double getTopFlyVelocity() {
    return topFlyEncoder.getVelocity();
   }

   @AutoLogOutput
   public double getTopFlySetpoint() {
    return topFlyController.getSetpoint();
   }

   @AutoLogOutput
   public double getFlywheelVelocity() {
    return mainFlyEncoder.getVelocity();
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
   public double getTopFlyCurrent() {
    return topFly.getOutputCurrent();
   }

   public Command runFlywheel(double volts){
    return run(() -> setFlywheelVoltage(volts));
   }

   public Command runTopFly(double volts){
    return run(() -> setTopFlyVoltage(volts));
   }

   public Command runShooter(double volts){
    return runFlywheel(volts).alongWith(runTopFly(volts));
   }

   public SysIdRoutine getMainFlySysidRoutine() {
    return new SysIdRoutine(new Config(Volts.of(.5).per(Second), Volts.of(7), null, (state)->{Logger.recordOutput("Shooter/mainFlySysidTestState", state.toString());}),
        new Mechanism(this::setFlywheelVoltage, null, this));
   }

   public SysIdRoutine getTopFlySysidRoutine() {
    return new SysIdRoutine(new Config(Volts.of(.5).per(Second), Volts.of(7), null, (state)->{Logger.recordOutput("Shooter/topFlySysidTestState", state.toString());}),
        new Mechanism(this::setTopFlyVoltage, null, this));
   }
   
}
