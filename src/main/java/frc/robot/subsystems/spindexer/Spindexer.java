package frc.robot.subsystems.spindexer;

import frc.robot.constants.SpindexerConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
    SparkFlex mainSpinner;
    SparkFlex feeder;
    SparkFlexConfig mainSpinnerConfig = new SparkFlexConfig();
    SparkFlexConfig feederConfig = new SparkFlexConfig();

    public Spindexer() {
        mainSpinner = new SparkFlex(SpindexerConstants.SPINNER_ID, MotorType.kBrushless);
        feeder = new SparkFlex(SpindexerConstants.FEEDER_ID, MotorType.kBrushless);
        configure();
    }

    public void configure()
    {
        mainSpinner.setCANTimeout(250);
        feeder.setCANTimeout(250);

         mainSpinnerConfig.smartCurrentLimit(SpindexerConstants.SPINNER_CURRENT_LIMIT);
         feederConfig.smartCurrentLimit(SpindexerConstants.FEEDER_CURRENT_LIMIT);
         mainSpinnerConfig.idleMode(IdleMode.kCoast);
         feederConfig.idleMode(IdleMode.kBrake);

        mainSpinner.configure(mainSpinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        mainSpinner.setCANTimeout(0);
        feeder.setCANTimeout(0);
    }

    public void setVoltageMainSpinner(double voltage) {
        mainSpinner.setVoltage(voltage);
    }

    public void setVoltageFeeder(double voltage) {
        feeder.setVoltage(voltage);
    }

    public void stop() {
        feeder.stopMotor();
        mainSpinner.stopMotor();
    }
    
}