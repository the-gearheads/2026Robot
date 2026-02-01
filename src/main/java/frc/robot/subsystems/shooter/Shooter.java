package frc.robot.subsystems.shooter;

import static frc.robot.constants.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase{

    PIDController pid;
    SimpleMotorFeedforward ff;

    String name;
    
    SparkFlex mainFly = new SparkFlex(MAIN_FLY_ID, MotorType.kBrushless);
    SparkFlexConfig mainFlyConfig = new SparkFlexConfig();
    RelativeEncoder mainFlyEncoder = mainFly.getEncoder(); 
    
    SparkFlex followerFly = new SparkFlex(FOLLOWER_FLY_ID, MotorType.kBrushless);
    SparkFlexConfig followerFlyConfig = new SparkFlexConfig();

    SparkFlex topFly = new SparkFlex(TOP_ROLLER_ID, MotorType.kBrushless);
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

    mainFlyConfig.smartCurrentLimit(65);
    mainFlyConfig.inverted(false);
    mainFlyConfig.idleMode(IdleMode.kBrake);
    
    mainFly.setCANTimeout(0);

    topFly.setCANTimeout(250);

    topFlyConfig.smartCurrentLimit(65);
    topFlyConfig.inverted(false);
    topFlyConfig.idleMode(IdleMode.kBrake);

    topFly.setCANTimeout(0);


   }

   public void setFlywheelVoltage(double volts) {
    mainFly.setVoltage(volts);
    Logger.recordOutput("Shooter/Flywheel/Volts", volts);
   }
   public void setTopRollerVoltage(double volts){
    topFly.setVoltage(volts);
    Logger.recordOutput("Shooter/Roller/Volts", volts);
   }
   public void setFlywheelVelocity(){}
   public void setTopRollerVelocity() {}

   public double getTopRollerVelocity() {
    return topFlyEncoder.getVelocity();
   }
   public double getFlywheelVelocity() {
    return mainFlyEncoder.getVelocity();
   }

   public double getFlywheelCurrent() {
    return mainFly.getOutputCurrent();
   }
   public double getTopRollerCurrent() {
    return topFly.getOutputCurrent();
   }

   public Command runFlywheel(double volts){
    return run(() -> setFlywheelVoltage(12));
   }
   public Command runTopRoller(double volts){
    return run(() -> setTopRollerVoltage(12));
   }
}
