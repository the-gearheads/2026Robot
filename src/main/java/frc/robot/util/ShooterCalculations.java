package frc.robot.util;


import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.MiscConstants.REAL_FUEL_PROCESSING_TIME;
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

    public static boolean isTimeToShoot(double timeOfFlight) {
        return HubTracker.isActive() || 
            (HubTracker.isActiveNext() && HubTracker.timeRemainingInCurrentShift().orElse(Seconds.of(25)).baseUnitMagnitude() < (timeOfFlight + REAL_FUEL_PROCESSING_TIME));
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
            Pose2d robotPose, ChassisSpeeds fieldRelSpeeds, double accelX, double accelY, double accelRot, AimingTarget aimingTarget, int iterations) {
                
        double vLx = fieldRelSpeeds.vxMetersPerSecond + (accelX * LATENCY_COMPENSATION);
        double vLy = fieldRelSpeeds.vyMetersPerSecond + (accelY * LATENCY_COMPENSATION);
        double omegaL = fieldRelSpeeds.omegaRadiansPerSecond + (accelRot * LATENCY_COMPENSATION);

        Translation2d posDelta = new Translation2d(
            vLx * LATENCY_COMPENSATION - (0.5 * accelX * Math.pow(LATENCY_COMPENSATION, 2)),
            vLy * LATENCY_COMPENSATION - (0.5 * accelY * Math.pow(LATENCY_COMPENSATION, 2))
        );
        Translation2d shotOrigin = robotPose.getTranslation().plus(posDelta);

        // (Translation + Whip)
        double rX = ShooterConstants.CENTER_BOT_TOSHOOT.getX();
        double rY = ShooterConstants.CENTER_BOT_TOSHOOT.getY();
        Translation2d totalShooterVel = new Translation2d(
            vLx + (-omegaL * rY), 
            vLy + (omegaL * rX)
        );

        // Initial Guess 
        double distance = shotOrigin.getDistance(aimingTarget.getFieldPosition());
        double baseTof = aimingTarget.getTimeOfFlight(distance);
        double t = applyLinearDragCompensation(baseTof, DRAG_CONSTANT);

        // Iterate the process, getting better time of flight estimations and updating the predicted target accordingly
        for (int i = 0; i < 3; i++) {
            // Find virtual target with current guess `t`
            Translation2d virtualTarget = predictTargetPos(
                aimingTarget.getFieldPosition(), fieldRelSpeeds, 
                accelX, accelY, accelRot, t, LATENCY_COMPENSATION
            );

            double distToVirtual = shotOrigin.getDistance(virtualTarget);
            
            // Calculate the actual dragged time it would take to hit that virtual target
            double currentBaseTof = aimingTarget.getTimeOfFlight(distToVirtual);
            double expectedDraggedTof = applyLinearDragCompensation(currentBaseTof, DRAG_CONSTANT);
            
            double error = t - expectedDraggedTof;

            Translation2d vectorToTarget = virtualTarget.minus(shotOrigin);
            Translation2d unitVector = vectorToTarget.div(distToVirtual);
            // fast the shooter is moving toward the virtual target
            double radialVelocity = (totalShooterVel.getX() * unitVector.getX()) + (totalShooterVel.getY() * unitVector.getY());

            // chain rule Derivatives
            double splineDerivative = aimingTarget.getTofDerivative(distToVirtual);
            double dragDerivative = Math.exp(-DRAG_CONSTANT * currentBaseTof);

            // f'(t) = 1 - (Drag' * Spline' * RadialVelocity)
            double fPrime = 1 - (dragDerivative * splineDerivative * radialVelocity);

            // Newton Update
            t = t - (error / fPrime);
        }  // to tune: forward and back from goal = latency comp, hits short = L too small
           //          sideways to the goal = Linear Drag, behind direction of travel = drag constant too low

        Translation2d finalVirtualTarget = predictTargetPos(aimingTarget.getFieldPosition(), fieldRelSpeeds, accelX, accelY, accelRot, t, LATENCY_COMPENSATION);
        VirtualTarget adjustedTarget = new VirtualTarget(aimingTarget, finalVirtualTarget);
        double finalDist = shotOrigin.getDistance(finalVirtualTarget);

        ShotData finalAdjustedShot = new ShotData(
                adjustedTarget.getFlywheelVel(finalDist),
                adjustedTarget.getHoodAngle(finalDist),
                t,
                finalVirtualTarget,
                adjustedTarget);
        Logger.recordOutput("ShooterCalculations/SOTMadjustedShot", finalAdjustedShot.toString());
        return finalAdjustedShot;
    }

    private static double applyLinearDragCompensation(double timeOfFlight, double dragConstant) {
        // again: https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/linear-drag.html
        double decayedToF = (1 - Math.exp(-dragConstant * timeOfFlight)) / dragConstant;
        return decayedToF;
    }
    
    // Move a target a set time in the future along a velocity defined by fieldSpeeds
    public static Translation2d predictTargetPos(Translation2d target, ChassisSpeeds fieldSpeeds, double accelX,
            double accelY, double accelRot, double tof, double latencySeconds) {
                
        // 1. Predict robot velocities at the moment of launch
        double vLx = fieldSpeeds.vxMetersPerSecond + (accelX * latencySeconds);
        double vLy = fieldSpeeds.vyMetersPerSecond + (accelY * latencySeconds);
        double omegaL = fieldSpeeds.omegaRadiansPerSecond + (accelRot * latencySeconds);

        // 2. Calculate the "Whip" (tangential velocity) at launch
        // rX and rY are the shooter's offset from robot center
        double rX = ShooterConstants.CENTER_BOT_TOSHOOT.getX();
        double rY = ShooterConstants.CENTER_BOT_TOSHOOT.getY();

        double whipVx = -omegaL * rY;
        double whipVy = omegaL * rX;

        // 3. Shift the target to create the virtual setpoint
        // Robot translation affects the shot for (Latency + ToF)
        // The "Whip" only affects the note during (ToF)
        double virtualX = target.getX() - (vLx * (tof + latencySeconds)) - (whipVx * tof);
        double virtualY = target.getY() - (vLy * (tof + latencySeconds)) - (whipVy * tof);

        return new Translation2d(virtualX, virtualY);
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
        Logger.recordOutput("HubTracker/ShiftTimeRemaining", HubTracker.timeRemainingInCurrentShift().orElse(Seconds.of(-1)));
        ObjectiveTracker.log(robotPose);
    }

    public record ShotData(double flywheelVel, Rotation2d hoodAngle, double timeOfFlight, Translation2d target, AimingTarget aimingTarget) {}
}
