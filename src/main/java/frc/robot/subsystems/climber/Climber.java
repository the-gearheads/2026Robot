package frc.robot.subsystems.climber;

import static frc.robot.constants.ClimberConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.AllianceFlipUtil;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class Climber extends SubsystemBase {
    SparkFlex climber = new SparkFlex(CLIMBER_ID,MotorType.kBrushless);
    RelativeEncoder climbEncoder = climber.getEncoder();
    //LaserCan laserCan = new LaserCan(LASERCAN_ID);

    SparkFlexConfig climbConfig = new SparkFlexConfig();

    public Climber() {
        configure();
        climbEncoder.setPosition(0);
        /* 
        laserCan.setRangingMode(LaserCan.RangingMode.SHORT); //UNCONFIGURED SETTINGS
        laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16)); //UNCONFIGURED SETTINGS
        laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS); //UNCONFIGURED SETTINGS
        */
    }

    @Override
    public void periodic() {
        if (climbEncoder.getPosition() < -10) {
            climbEncoder.setPosition(0);
        }
    }
    
    public void configure(){
        climbConfig.smartCurrentLimit(CLIMBER_CURRENT_LIMIT);
        climbConfig.idleMode(IdleMode.kBrake);
        climbConfig.inverted(false);
        climbConfig.softLimit.forwardSoftLimit(ClimberConstants.MAX_CLIMBER_POS);
        climbConfig.softLimit.reverseSoftLimit(ClimberConstants.MIN_CLIMBER_POS);
        climbConfig.softLimit.reverseSoftLimitEnabled(true);
        climbConfig.softLimit.forwardSoftLimitEnabled(true);
        climber.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        climbConfig.encoder.quadratureMeasurementPeriod(10);
        climbConfig.encoder.quadratureAverageDepth(2); 
    }


    public void setClimberVoltage(double voltage) {
        climber.setVoltage(voltage);
    }

    @AutoLogOutput
    public double getClimbPosition() {
        return climbEncoder.getPosition();
    }

    @AutoLogOutput
    public double getClimberVoltage() {
        return climber.getAppliedOutput() * climber.getBusVoltage();
    }

    public Command climberUp() {
        return this.run(()->{
            setClimberVoltage(ClimberConstants.CLIMB_UP_VOLTAGE);
        }).until(()->{return getClimbPosition()>=ClimberConstants.MAX_CLIMBER_POS;}).finallyDo(()->{
            setClimberVoltage(0);
        });
    }
    
    public Command climberDown() {
        return this.run(()->{
            setClimberVoltage(-ClimberConstants.CLIMB_DOWN_VOLTAGE);
        }).until(()->{return getClimbPosition()<=ClimberConstants.MIN_CLIMBER_POS;}).finallyDo(()->{
            setClimberVoltage(0);
        });
    }

    public void setPosition(double position) {
        climbEncoder.setPosition(position);
    }

   /* @AutoLogOutput
    public boolean isClimbingPole() {
        //return ((laserCan.getMeasurement().distance_mm)/1000) < ClimberConstants.CLIMBING_POLE_PROXIMITY_THRESHOLD;
    }
    */

    /*
    @AutoLogOutput
    public double getColorProximity() {
       //return ((laserCan.getMeasurement().distance_mm)/1000);
    }
    */



    public Command autoClimb(Swerve swerve) {
        Pose2d climbingPose;
        double drivingVelocity;
        if(AllianceFlipUtil.applyY(swerve.getPose().getY()) < FieldConstants.fieldWidth/2.0) {
            climbingPose = AllianceFlipUtil.apply(CLIMB_RIGHT_POSE);
            drivingVelocity = CLIMB_SWEEP_SPEED;
        } else {
            climbingPose = AllianceFlipUtil.apply(CLIMB_LEFT_POSE);
            drivingVelocity = -CLIMB_SWEEP_SPEED;
        }

        return Commands.sequence(
            climberUp(),
            Commands.sequence(
                swerve.pathFindToPose(climbingPose),
                swerve.run(() -> {
                    swerve.drive(new ChassisSpeeds(-CLIMB_IN_SPEED, drivingVelocity, 0), climbingPose.getRotation());
                }).withTimeout(2),
                swerve.runOnce(() -> {
                            swerve.drive(new ChassisSpeeds(0, 0, 0), climbingPose.getRotation());
                })
            )
        ).andThen(Commands.sequence(
                // swerve.run(() -> {
                //     swerve.drive(new ChassisSpeeds(-CLIMB_IN_SPEED, 0, 0), climbingPose.getRotation());
                // }).until(() -> {
                //     return getColorProximity() <= AUTOCLIMB_DOWN_PROXIMITY;
                // }).withTimeout(1.5),

                // swerve.runOnce(() -> {
                //     swerve.drive(new ChassisSpeeds(0, 0, 0));
                // }),

                climberDown()
        ));
    }
}
