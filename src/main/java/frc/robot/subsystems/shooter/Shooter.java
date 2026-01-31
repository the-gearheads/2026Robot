package frc.robot.subsystems.shooter;

import static frc.robot.constants.ShooterConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

   public Shooter() {
    
   }
   
   @Override
   public void periodic(){}

   public void configure() {
    mainFly.setCANTimeout(250);

    mainFlyConfig.smartCurrentLimit(65);
    mainFlyConfig.inverted(false);
    mainFlyConfig.idleMode(IdleMode.kBrake);
    
    mainFly.setCANTimeout(0);
   }

   public void setFlywheelVoltage(double volts) {
    mainFly.setVoltage(volts);
   }
   public void setTopRollerVoltage(double volts){
    topFly.setVoltage(volts);
   }
   public void setFlywheelVelocity(){}
   public void setTopRollerVelocity() {}

   public double getTopRollerVelocity() {}
   public double getFlywheelVelocity() {}

   public double getFlywheelCurrent() {}
   public double getTopRollerCurrent() {}
}
