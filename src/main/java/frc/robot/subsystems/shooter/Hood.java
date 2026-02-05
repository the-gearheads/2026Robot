package frc.robot.subsystems.shooter;


import static frc.robot.constants.ShooterConstants.HOOD_PID;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.HOOD_FEEDFORWARD;
import static frc.robot.constants.ShooterConstants.HOOD_MOTOR_ID;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
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

public class Hood extends SubsystemBase {
    SparkFlex hood = new SparkFlex(HOOD_MOTOR_ID, MotorType.kBrushless);
    SparkClosedLoopController hoodController = hood.getClosedLoopController();
    SparkFlexConfig hoodConfig = new SparkFlexConfig();
    RelativeEncoder hoodEncoder = hood.getEncoder();

    
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
        hoodConfig.voltageCompensation(12);
        hoodConfig.closedLoop.pid(HOOD_PID[0], HOOD_PID[1], HOOD_PID[2]);
        hoodConfig.closedLoop.feedForward.kS(HOOD_FEEDFORWARD.getKs());
        hoodConfig.closedLoop.feedForward.kV(HOOD_FEEDFORWARD.getKv());
        hoodConfig.closedLoop.feedForward.kA(HOOD_FEEDFORWARD.getKa());
        hoodConfig.closedLoop.feedForward.kG(HOOD_FEEDFORWARD.getKg());
    
        hood.setCANTimeout(0);
    }

    public void setHoodVoltage(double volts){
        hoodController.setSetpoint(volts, ControlType.kVoltage);
    }
    
    public void setHoodVoltage(Voltage volts){
        setHoodVoltage(volts.magnitude());
    }


    @AutoLogOutput
    public double getHoodVelocity(){
       return hoodEncoder.getVelocity();
    }

    public Command hoodManual(double volts){
        return run(() -> setHoodVoltage(volts));
    }

    public SysIdRoutine getMainFlySysidRoutine() {
    return new SysIdRoutine(new Config(Volts.of(.5).per(Second), Volts.of(7), null, (state)->{Logger.recordOutput("Hood/SysidTestState", state.toString());}),
        new Mechanism(this::setHoodVoltage, null, this));
   }


}