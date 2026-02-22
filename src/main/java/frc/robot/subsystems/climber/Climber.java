package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class Climber extends SubsystemBase {
    public SparkFlex climber = new SparkFlex(60,MotorType.kBrushless);
    public RelativeEncoder climbEncoder = climber.getEncoder();
    public SparkFlexConfig climbConfig = new SparkFlexConfig();
    
    public Climber() {
        configure();
    }
    
    public void configure(){
        climber.setCANTimeout(250);

        climbConfig.smartCurrentLimit(60);
        climbConfig.idleMode(IdleMode.kBrake);
        climber.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        climber.setCANTimeout(0);
    }


    public void setClimberVoltage(double voltage) {
        climber.setVoltage(voltage);
    }

    public double getClimbPosition() {
        return climbEncoder.getPosition();
    }

    
    
}
