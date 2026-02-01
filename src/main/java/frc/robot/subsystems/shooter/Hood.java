package frc.robot.subsystems.shooter;

import static frc.robot.constants.ShooterConstants.HOOD_CONSTRAINTS;
import static frc.robot.constants.ShooterConstants.HOOD_MOTOR_ID;
import static frc.robot.constants.ShooterConstants.HOOD_PID;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    SparkFlex hood = new SparkFlex(HOOD_MOTOR_ID, MotorType.kBrushless);
    SparkFlexConfig hoodConfig = new SparkFlexConfig();
    RelativeEncoder hoodEncoder = hood.getEncoder();
    ProfiledPIDController pid = new ProfiledPIDController(HOOD_PID[0], HOOD_PID[1], HOOD_PID[2], HOOD_CONSTRAINTS);  

    double ff;

    
    public Hood() {
        configure();
    }

    @Override
    public void periodic(){}

    

    public void configure(){
        hood.setCANTimeout(250);

        hoodConfig.smartCurrentLimit(65);
        hoodConfig.inverted(false);
        hoodConfig.idleMode(IdleMode.kBrake);
    
        hood.setCANTimeout(0);
    }

    public void setHoodVoltage(double volts){
        hood.setVoltage(volts);
        Logger.recordOutput("Shooter/Hood/Volts", volts);
    }

    public void getHoodVelocity(){
        hoodEncoder.getVelocity();
    }

    public Command hoodManual(double volts){
        return run(() -> setHoodVoltage(volts));
    }



}