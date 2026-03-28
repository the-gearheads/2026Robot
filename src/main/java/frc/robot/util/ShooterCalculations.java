package frc.robot.util;


import static frc.robot.constants.ShooterConstants.DRAG_CONSTANT;
import static frc.robot.constants.ShooterConstants.HOOD_MIN_ANGLE;
import static frc.robot.constants.ShooterConstants.HOOD_MOVING_TOLERANCE;
import static frc.robot.constants.ShooterConstants.LATENCY_COMPENSATION;
import static frc.robot.constants.SwerveConstants.YAW_ALIGN_TOLERANCE;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.AimingManager;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.targets.VirtualTarget;

public class ShooterCalculations {
    public static ArrayList<Double> HubDists = new ArrayList<>();
    public static ArrayList<Double> ShooterSpeeds = new ArrayList<>();
    public static ArrayList<Rotation2d> HoodAngles = new ArrayList<>();

    public static boolean readyToShoot(Pose2d robotPose, Hood hood, Shooter shooter, AimingTarget overrideTarget) {
        ShotData shot;
        if (overrideTarget != null) {
            if (ObjectiveTracker.HUB == overrideTarget) {
                shot = AimingManager.latestHubShot;
            } else {
                shot = AimingManager.latestFeedShot;
            }
        } else {
            shot = AimingManager.latestShot; 
        }
        
        boolean yawReady = MathUtil.isNear(getYawToTarget(robotPose, shot.aimingTarget).getRadians(), robotPose.getRotation().getRadians(), YAW_ALIGN_TOLERANCE.getRadians());
        boolean hoodReady = hood.atAngle(shot.hoodAngle(), HOOD_MOVING_TOLERANCE);
        boolean shooterReady = shooter.atSpeed(shot.flywheelVel());
        Logger.recordOutput("ShooterCalculations/yawReady", yawReady);
        Logger.recordOutput("ShooterCalculations/hoodReady", hoodReady);
        Logger.recordOutput("ShooterCalculations/shooterReady", shooterReady);
        return hoodReady && shooterReady && yawReady;
    }

    public static boolean readyToShoot(Pose2d robotPose, Hood hood, Shooter shooter) {
        return readyToShoot(robotPose, hood, shooter, null);
    }
    
    public static ShotData calculateStillShot(Pose2d robotPose, AimingTarget aimingTarget) {
        double targetDist = getDistanceToTarget(robotPose, aimingTarget.getFieldPosition());
        return new ShotData(
            aimingTarget.getFlywheelVel(targetDist),
            aimingTarget.getHoodAngle(targetDist),
            aimingTarget.getTimeOfFlight(targetDist),
            aimingTarget.getFieldPosition(),
            aimingTarget
        );
    }

    public static ShotData applyTrenchAvoidance(ShotData baseShot, Pose2d robotPose, ChassisSpeeds fieldRelSpeeds) {
        Rectangle2d[] badRectangles = getTrenchAvoidanceRectanlges(robotPose, fieldRelSpeeds);
        for (Rectangle2d zone : badRectangles) {
            if (zone.contains(robotPose.getTranslation())) {
                return new ShotData(baseShot.flywheelVel, HOOD_MIN_ANGLE, baseShot.timeOfFlight, baseShot.target, baseShot.aimingTarget);
            }
        }
        return baseShot;
    }

    public static double getDistanceToTarget(Pose2d robotPose, Translation2d target) {
        return target.getDistance(getShooterPosition(robotPose).getTranslation());
    }

    public static Rotation2d getYawToTarget(Pose2d robotPose, AimingTarget aimingTarget) {
        Translation2d targetPosition = aimingTarget.getFieldPosition();
        Rotation2d targetAngle = targetPosition.minus(getShooterPosition(robotPose).getTranslation()).getAngle();
        return targetAngle;  
    } 

    public static ShotData iterativeMovingShot(
            Pose2d robotPose, ChassisSpeeds fieldRelSpeeds, AimingTarget aimingTarget, int iterations) {
                
        // Perform initial estimation (assuming unmoving robot) to get time of flight estimate
        double distance = getDistanceToTarget(robotPose, aimingTarget.getFieldPosition());
        double timeOfFlight = aimingTarget.getTimeOfFlight(distance);
        Translation2d movingTargetPos = aimingTarget.getFieldPosition();
        
        // Iterate the process, getting better time of flight estimations and updating the predicted target accordingly
        for (int i = 0; i < iterations; i++) {
            movingTargetPos = predictTargetPos(aimingTarget.getFieldPosition(), fieldRelSpeeds, timeOfFlight);
            timeOfFlight = aimingTarget.getTimeOfFlight(getDistanceToTarget(robotPose, movingTargetPos));  // use the table for whatever the basic table is, but the distance is changing
            timeOfFlight = applyLinearDragCompensation(timeOfFlight, DRAG_CONSTANT);
            timeOfFlight += LATENCY_COMPENSATION;
        }  // to tune: forward and back from goal = latency comp, hits short = L too small
           //          sideways to the goal = Linear Drag, behind direction of travel = drag constant too low

        VirtualTarget adjustedTarget = new VirtualTarget(aimingTarget, movingTargetPos);
        ShotData adjustedShot = calculateStillShot(robotPose, adjustedTarget);

        Logger.recordOutput("ShooterCalculations/SOTMadjustedShot", adjustedShot.toString());
        return adjustedShot;
    }

    private static double applyLinearDragCompensation(double timeOfFlight, double dragConstant) {
        // again: https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/linear-drag.html
        double decayedToF = (1 - Math.exp(-dragConstant * timeOfFlight)) / dragConstant;
        return decayedToF;
    }
    
    // Move a target a set time in the future along a velocity defined by fieldSpeeds
    public static Translation2d predictTargetPos(Translation2d target, ChassisSpeeds fieldSpeeds, double tofSeconds) {
        double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * tofSeconds;
        double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * tofSeconds;

        return new Translation2d(predictedX, predictedY);
    }


    /**
     *  IF YOU'RE CALLING THIS, YOU'RE PROBABLY DOING SOMETHING WRONG AND IT'S ALREADY BEEN CALLED FURTHER DOWN THE LINE
     * @param robotPose
     * @return Position of shooter on field
     */
    private static Pose2d getShooterPosition(Pose2d robotPose) {
        Translation2d shooterDistance = ShooterConstants.CENTER_BOT_TOSHOOT.toTranslation2d();
        
        Pose2d shooterPose = robotPose.plus(new Transform2d(shooterDistance, new Rotation2d(0)));
        Logger.recordOutput("ShooterCalculations/shooterPosition", shooterPose);
        return shooterPose;
    }

    private static Rectangle2d[] getTrenchAvoidanceRectanlges(Pose2d pose, ChassisSpeeds fieldRelativeRobotSpeed) {
        Translation2d[] hubSideCorners = {
            new Translation2d(4.63, 2.1),
            new Translation2d(4.63, 6),
            new Translation2d(11.918, 2.1),
            new Translation2d(11.918, 6)
        };

        Translation2d[] WallSideCorners = {  // 0.3 is a tiny bit of tolerance in case localization is outside of the field
            new Translation2d(4.63, 0 - 5),
            new Translation2d(4.63, FieldConstants.fieldWidth + 5),
            new Translation2d(11.918, 0 - 5),
            new Translation2d(11.918, FieldConstants.fieldWidth + 5)
        };

        double rectWidth = (Math.abs(fieldRelativeRobotSpeed.vxMetersPerSecond) * 0.6) + Units.inchesToMeters(45);
        Rectangle2d[] scaryZones = new Rectangle2d[hubSideCorners.length];
        for(int i=0; i<hubSideCorners.length; i++) {
            Rectangle2d currentZone = new Rectangle2d(hubSideCorners[i].plus(new Translation2d(rectWidth, 0)), WallSideCorners[i].plus(new Translation2d(-rectWidth, 0)));
            Logger.recordOutput("ShooterCalculations/zoneCorners" + i, hubSideCorners[i].plus(new Translation2d(rectWidth, 0)), WallSideCorners[i].plus(new Translation2d(-rectWidth, 0)));
            scaryZones[i] = currentZone;
        }
        return scaryZones;
    }

    public static void log(Pose2d robotPose) {
        Logger.recordOutput("ShooterCalculations/HubDistance", getDistanceToTarget(robotPose, ObjectiveTracker.HUB.getFieldPosition()));
        ObjectiveTracker.log(robotPose);
    }

    public record ShotData(double flywheelVel, Rotation2d hoodAngle, double timeOfFlight, Translation2d target, AimingTarget aimingTarget) {}
}
