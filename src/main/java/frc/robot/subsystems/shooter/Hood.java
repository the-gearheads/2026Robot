package frc.robot.subsystems.shooter;

import static frc.robot.constants.ShooterConstants.HOOD_MOTOR_ID;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    SparkFlex hood = new SparkFlex(HOOD_MOTOR_ID, MotorType.kBrushless);
    SparkFlexConfig hoodConfig = new SparkFlexConfig();
    RelativeEncoder hoodEncoder = hood.getEncoder();
    
    public Hood() {
        
    }

}