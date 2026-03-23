package frc.robot.subsystems.shooter;


import static frc.robot.constants.ShooterConstants.HOOD_POS_FACTOR;
import static frc.robot.constants.ShooterConstants.HOOD_UP_KS;
import static frc.robot.constants.ShooterConstants.HOOD_VEL_FACTOR;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.HOOD_ANGLE_TOLERANCE;
import static frc.robot.constants.ShooterConstants.HOOD_CONSTRAINTS;
import static frc.robot.constants.ShooterConstants.HOOD_DOWN_KS;
import static frc.robot.constants.ShooterConstants.HOOD_FEEDFORWARD;
import static frc.robot.constants.ShooterConstants.HOOD_I_ZONE;
import static frc.robot.constants.ShooterConstants.HOOD_MAX_ANGLE;
import static frc.robot.constants.ShooterConstants.HOOD_MAX_I_ACCUM;
import static frc.robot.constants.ShooterConstants.HOOD_MAX_SYSID_ANGLE;
import static frc.robot.constants.ShooterConstants.HOOD_MIN_ANGLE;
import static frc.robot.constants.ShooterConstants.HOOD_MIN_SYSID_ANGLE;
import static frc.robot.constants.ShooterConstants.HOOD_MOTOR_ID;
import static frc.robot.constants.ShooterConstants.HOOD_PID;

import frc.robot.subsystems.swerve.Swerve;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.util.ShooterCalculations;

public class Hood extends SubsystemBase {
    SparkFlex hood = new SparkFlex(HOOD_MOTOR_ID, MotorType.kBrushless);
    SparkClosedLoopController hoodController = hood.getClosedLoopController();
    SparkFlexConfig hoodConfig = new SparkFlexConfig();
    RelativeEncoder hoodEncoder = hood.getEncoder();
    TrapezoidProfile profile = new TrapezoidProfile(HOOD_CONSTRAINTS);
    Rotation2d targetAngle = new Rotation2d();
    TrapezoidProfile.State profileSetpoint;
    TrapezoidProfile.State lastSetpoint;

    PIDController hoodPID;
    
    @AutoLogOutput
    boolean isManualMode = false;
    public Hood() {
        configure();
        hoodPID = new PIDController(HOOD_PID[0], HOOD_PID[1], HOOD_PID[2]);
        hoodPID.setIZone(HOOD_I_ZONE);
        hoodPID.setIntegratorRange(-HOOD_MAX_I_ACCUM, HOOD_MAX_I_ACCUM);

        SmartDashboard.putNumber("Hood/FF/Kv", HOOD_FEEDFORWARD.getKv());
        SmartDashboard.putNumber("Hood/FF/Ka", HOOD_FEEDFORWARD.getKa());
        SmartDashboard.putNumber("Hood/PID/P", HOOD_PID[0]);
        SmartDashboard.putNumber("Hood/PID/I", HOOD_PID[1]);
        SmartDashboard.putNumber("Hood/PID/D", HOOD_PID[2]);

        hoodEncoder.setPosition(0);
        profileSetpoint = new State(getAngle().getRadians(), 0);
    }

    @Override
    public void periodic() {
        HOOD_FEEDFORWARD.setKv(SmartDashboard.getNumber("Hood/FF/Kv", HOOD_FEEDFORWARD.getKv()));
        HOOD_FEEDFORWARD.setKa(SmartDashboard.getNumber("Hood/FF/Ka", HOOD_FEEDFORWARD.getKa()));
        hoodPID.setP(SmartDashboard.getNumber("Hood/PID/P", HOOD_PID[0]));
        hoodPID.setI(SmartDashboard.getNumber("Hood/PID/I", HOOD_PID[1]));
        hoodPID.setD(SmartDashboard.getNumber("Hood/PID/D", HOOD_PID[2]));

        if (!isManualMode) {
            lastSetpoint = profileSetpoint;

            profileSetpoint = profile.calculate(0.02, profileSetpoint, new State(targetAngle.getRadians(), 0));

            double truePosError = targetAngle.getRadians() - getAngle().getRadians();  // because its only used for applying ks, should use targetAngle not profileSetpoint
            double ff = HOOD_FEEDFORWARD.calculateWithVelocities(lastSetpoint.velocity, profileSetpoint.velocity);

            // Apply kS whenever meaningfully off target - during AND after the profile
            if (Math.abs(truePosError) > HOOD_ANGLE_TOLERANCE.getRadians()) {
                // if (Math.abs(truePosError) < HOOD_ANGLE_TOLERANCE.getRadians()*1.5) {
                //     ff += (truePosError > 0 ? HOOD_UP_KS/2.0 : HOOD_DOWN_KS/2.0);
                //     Logger.recordOutput("Hood/currentKs", (truePosError > 0 ? HOOD_UP_KS/2.0 : HOOD_DOWN_KS/2.0));
                // } else {
                    ff += (truePosError > 0 ? HOOD_UP_KS : HOOD_DOWN_KS);
                    Logger.recordOutput("Hood/currentKs", (truePosError > 0 ? HOOD_UP_KS : HOOD_DOWN_KS));
                // }
            }

            double pid = hoodPID.calculate(getAngle().getRadians(), profileSetpoint.position);
            hood.setVoltage(pid + ff);

            Logger.recordOutput("Hood/profileSetpoint", profileSetpoint);
            Logger.recordOutput("Hood/ff", ff);
            Logger.recordOutput("Hood/pid", pid);
            Logger.recordOutput("Hood/posError", Rotation2d.fromRadians(truePosError));

        } else {
            profileSetpoint = new State(getAngle().getRadians(), getVelocity());
        }
    }

    

    public void configure() {
        hoodConfig.encoder.quadratureMeasurementPeriod(10);
        hoodConfig.encoder.quadratureAverageDepth(2); 
        hoodConfig.smartCurrentLimit(65);
      
        hoodConfig.inverted(false);
        hoodConfig.idleMode(IdleMode.kBrake);
        hoodConfig.voltageCompensation(12);

        hoodConfig.closedLoop.pid(0, 0, 0, ClosedLoopSlot.kSlot0);
        hoodConfig.closedLoop.outputRange(-1, 1);

        hoodConfig.signals.primaryEncoderPositionPeriodMs(10);
        hoodConfig.signals.primaryEncoderVelocityPeriodMs(10);

        hoodConfig.encoder.positionConversionFactor(HOOD_POS_FACTOR);  
        hoodConfig.encoder.velocityConversionFactor(HOOD_VEL_FACTOR);
    
        hood.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setVoltage(double volts){
        isManualMode = true;
        if (getAngle().getRadians() > HOOD_MAX_ANGLE && volts > 0) {
            hoodController.setSetpoint(0, ControlType.kVoltage);
        } else if (getAngle().getRadians() < HOOD_MIN_ANGLE && volts < 0) {
            hoodController.setSetpoint(0, ControlType.kVoltage);
        } else {
            hoodController.setSetpoint(volts, ControlType.kVoltage);
        }
    }                                                                       
    
    public void setVoltage(Voltage volts){
        setVoltage(volts.magnitude());
    }

    public void setAngle(Rotation2d angle) {
        if (isManualMode) {
            profileSetpoint = new State(getAngle().getRadians(), getVelocity());
            isManualMode = false;
        }
        targetAngle = angle;
        Logger.recordOutput("Hood/LastGoalAngle", angle);
    }

    public Command NTVoltageCommand() {
        SmartDashboard.putNumber("Hood/manVoltage", 0);
        return this.run(()->{
            this.setVoltage(SmartDashboard.getNumber("Hood/manVoltage", 0));
        });
    }

    public Command setAngleCommand(Rotation2d angle) {
        return this.run(() -> {
            this.setAngle(angle);
        });
    }

    public Command setAngleCommand(Rotation2d angle, boolean ends) {
        if (!ends) return setAngleCommand(angle);
        return this.run(() -> {
            this.setAngle(angle);
        }).until(() -> {return atAngle(angle);});
    }

    public boolean atAngle(Rotation2d angle) {
        return MathUtil.isNear(angle.getRadians(), getAngle().getRadians(), HOOD_ANGLE_TOLERANCE.getRadians());
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
    public double getMotorControllerSetpoint() {
        return hoodController.getSetpoint();
    }

    @AutoLogOutput
    public double getCurrent() {
        return hood.getOutputCurrent();
    }

    public Command hoodManual(double volts){
        return run(() -> setVoltage(volts)).finallyDo(() -> setVoltage(0));
    }

    public Command hoodHome() { // bypasses limits
        Command stage1 = this.run(()->{
            this.isManualMode = true;
            this.hood.setVoltage(-1.5);
            this.targetAngle = getAngle();  // shouldn't need this but if manual mode breaks or something
        }).finallyDo(()->{
            this.hood.setVoltage(0);
        }).withTimeout(2);

        Command dwell = this.run(()->{
            // just in case, theoretically should not be needed
            this.isManualMode = true;
            this.hood.setVoltage(0);
        }).withTimeout(0.2);

        Command stage2 = this.runOnce(()->{
            this.hoodEncoder.setPosition(0);
            this.targetAngle = Rotation2d.kZero;
            this.setAngle(Rotation2d.kZero);
            profileSetpoint = new State(0, 0);
            lastSetpoint = new State(0, 0);
            this.isManualMode = false;
        });

        return Commands.sequence(
            stage1,
            dwell,
            stage2
        );
    }

    public SysIdRoutine getSysIdRoutine() {
        return new SysIdRoutine(new Config(Volts.of(.35).per(Second), Volts.of(1), null, (state)->{Logger.recordOutput("Hood/SysidTestState", state.toString());}),
            new Mechanism(this::setVoltage, null, this));
    }

    public boolean forwardSysIdLimit() {
        return getAngle().getRadians() >= HOOD_MAX_SYSID_ANGLE;
    } 

    public boolean reverseSysIdLimit() {
        return getAngle().getRadians() <= HOOD_MIN_SYSID_ANGLE;
    }

    public Command setObjectiveAngleCommand(Swerve swerve) {
        return this.run(() -> {
            this.setAngle(ShooterCalculations.getObjectiveHoodAngle(swerve));
        });
    }

    public Command setAngleFeed(Swerve swerve) {
        return this.run(() -> {
            this.setAngle(ShooterCalculations.getFeederHoodAngle(swerve));
        });
    }

    public Command setAngleHub(Swerve swerve) {
        return this.run(() -> {
            this.setAngle(ShooterCalculations.getHubHoodAngle(swerve));
        });
    }
}