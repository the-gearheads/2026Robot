package frc.robot;

import static frc.robot.constants.MiscConstants.SOTM_ITERATIONS;
import static frc.robot.constants.ShooterConstants.HOOD_ANGLE_ADJUSTMENT;
import static frc.robot.constants.ShooterConstants.SHOOTER_VEL_ADJUSTMENT;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.AimingTarget;
import frc.robot.util.ObjectiveTracker;
import frc.robot.util.ShooterCalculations;
import frc.robot.util.ShooterCalculations.ShotData;


/*
 * God.
 */
public class AimingManager {
    public static ShotData latestShot;
    public static ShotData lastestHubShot;
    public static ShotData latestFeedShot;
    public static boolean USE_SOTM = true;

    public static void update(Pose2d robotPose, ChassisSpeeds fieldRelSpeeds) {
        AimingTarget ObjectiveTarget = ObjectiveTracker.getObjective(robotPose);

        AimingTarget hubTarget = ObjectiveTracker.HUB;
        ShotData hubSotmShot = ShooterCalculations.iterativeMovingShot(robotPose, fieldRelSpeeds, hubTarget, SOTM_ITERATIONS);
        ShotData hubStillShot = ShooterCalculations.calculateStillShot(robotPose, hubTarget);

        AimingTarget feedTarget = ObjectiveTracker.getFeedingObjective(robotPose);
        ShotData feedSotmShot = ShooterCalculations.iterativeMovingShot(robotPose, fieldRelSpeeds, feedTarget, SOTM_ITERATIONS);
        ShotData feedStillShot = ShooterCalculations.calculateStillShot(robotPose, feedTarget);

        if (ObjectiveTarget != ObjectiveTracker.HUB) {
            latestShot = applySafeties(USE_SOTM ? feedSotmShot : feedStillShot, robotPose, fieldRelSpeeds);
        } else {
            latestShot = applySafeties(USE_SOTM ? hubSotmShot : hubStillShot, robotPose, fieldRelSpeeds);
        }

        lastestHubShot = applySafeties(USE_SOTM ? hubSotmShot : hubStillShot, robotPose, fieldRelSpeeds);
        latestFeedShot = applySafeties(USE_SOTM ? feedSotmShot : feedStillShot, robotPose, fieldRelSpeeds);

        Logger.recordOutput("AimingManager/targetPosition", ObjectiveTarget.getFieldPosition());
        Logger.recordOutput("AimingManager/latestShot", latestShot);
        Logger.recordOutput("AimingManager/hubShotSotm", hubSotmShot);
        Logger.recordOutput("AimingManager/hubShotStill", hubStillShot);
        Logger.recordOutput("AimingManager/feedSotmShot", feedSotmShot);
        Logger.recordOutput("AimingManager/feedStillShot", feedStillShot);
    }

    private static ShotData applySafeties(ShotData baseShot, Pose2d robotPose, ChassisSpeeds fieldRelSpeeds) {
        ShotData trenchAvoidanceShot = ShooterCalculations.applyTrenchAvoidance(baseShot, robotPose, fieldRelSpeeds);
        return new ShotData(
            trenchAvoidanceShot.flywheelVel() + SHOOTER_VEL_ADJUSTMENT,
            trenchAvoidanceShot.hoodAngle().plus(HOOD_ANGLE_ADJUSTMENT),
            trenchAvoidanceShot.timeOfFlight(),
            trenchAvoidanceShot.target(),
            trenchAvoidanceShot.aimingTarget()
        );
    } 
}
