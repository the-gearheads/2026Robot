package frc.robot.subsystems.shooter;


import static frc.robot.constants.ShooterConstants.HOOD_PID;
import static frc.robot.constants.ShooterConstants.HOOD_POS_FACTOR;
import static frc.robot.constants.ShooterConstants.HOOD_VEL_FACTOR;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.HOOD_FEEDFORWARD;
import static frc.robot.constants.ShooterConstants.HOOD_MAX_SYSID_ANGLE;
import static frc.robot.constants.ShooterConstants.HOOD_MIN_SYSID_ANGLE;
import static frc.robot.constants.ShooterConstants.HOOD_MOTOR_ID;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
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
    TrapezoidProfile profile = new TrapezoidProfile(new Constraints(
        Units.degreesToRadians(100), // per second; max vel
        Units.degreesToRadians(50)  //  per sec^2; max accel
    ));

    public Hood() {
        configure();
        hoodEncoder.setPosition(0);
    }

    @Override
    public void periodic() {}

    

    public void configure() {
        hood.setCANTimeout(10);

        hoodConfig.encoder.quadratureMeasurementPeriod(10);
        hoodConfig.encoder.quadratureAverageDepth(2); 
        hoodConfig.smartCurrentLimit(65);
      
        hoodConfig.inverted(false);
        hoodConfig.idleMode(IdleMode.kBrake);
        hoodConfig.voltageCompensation(12);
        hoodConfig.closedLoop.pid(HOOD_PID[0] / 12.0, HOOD_PID[1] / 12.0, HOOD_PID[2] / 12.0, ClosedLoopSlot.kSlot0);
        // hoodConfig.closedLoop.feedForward.kS(HOOD_FEEDFORWARD.getKs(), ClosedLoopSlot.kSlot0);
        // hoodConfig.closedLoop.feedForward.kV(HOOD_FEEDFORWARD.getKv(), ClosedLoopSlot.kSlot0);
        // hoodConfig.closedLoop.feedForward.kA(HOOD_FEEDFORWARD.getKa(), ClosedLoopSlot.kSlot0);
        // hoodConfig.closedLoop.feedForward.kG(HOOD_FEEDFORWARD.getKg(), ClosedLoopSlot.kSlot0);
        // hoodConfig.closedLoop.feedForward.kCos(HOOD_FEEDFORWARD.getKg(), ClosedLoopSlot.kSlot0);
        // hoodConfig.closedLoop.feedForward.kCosRatio(idek);
        hoodConfig.closedLoop.outputRange(-1, 1);

        hoodConfig.encoder.positionConversionFactor(HOOD_POS_FACTOR);  
        hoodConfig.encoder.velocityConversionFactor(HOOD_VEL_FACTOR);
    
        hood.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hood.setCANTimeout(0);
    }

    public void setVoltage(double volts){
        hoodController.setSetpoint(volts, ControlType.kVoltage, ClosedLoopSlot.kSlot0);
    }                                                                       
    
    public void setVoltage(Voltage volts){
        setVoltage(volts.magnitude());
    }

    public void setAngle(Rotation2d angle) {
        Logger.recordOutput("Hood/LastGoalAngle", angle);
        State setpoint = profile.calculate(0.02, new State(getAngle().getRadians(), getVelocity()), new State(angle.getRadians(), 0));
        double ff = HOOD_FEEDFORWARD.calculate(setpoint.position, setpoint.velocity);
        Logger.recordOutput("Hood/ff", ff);
        hoodController.setSetpoint(angle.getRadians(), ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
    }

    @AutoLogOutput
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(hoodEncoder.getPosition());
    }

    @AutoLogOutput
    public double getVelocity(){
       return hoodEncoder.getVelocity();
    }

    @AutoLogOutput
    public ControlType getControlType() {
        return hoodController.getControlType();
    }

    @AutoLogOutput
    public double getSetpoint() {
        return hoodController.getSetpoint();
    }

    @AutoLogOutput
    public double getCurrent() {
        return hood.getOutputCurrent();
    }

    public Command hoodManual(double volts){
        return run(() -> setVoltage(volts)).finallyDo(() -> setVoltage(0));
    }

    public Command hoodHome (){
        return runEnd(() -> setVoltage(-2), ()->{hoodEncoder.setPosition(0);}).withTimeout(2);
    }

    public SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(new Config(Volts.of(.35).per(Second), Volts.of(2), null, (state)->{Logger.recordOutput("Hood/SysidTestState", state.toString());}),
            new Mechanism(this::setVoltage, null, this));
    }

    public boolean forwardSysIdLimit() {
        return getAngle().getRadians() >= HOOD_MAX_SYSID_ANGLE;
    } 

    public boolean reverseSysIdLimit() {
        return getAngle().getRadians() <= HOOD_MIN_SYSID_ANGLE;
    }
}