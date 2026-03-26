package frc.robot;

import static frc.robot.constants.MiscConstants.SOTM_ITERATIONS;

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
            latestShot = USE_SOTM ? feedSotmShot : feedStillShot;
        } else {
            latestShot = USE_SOTM ? hubSotmShot : hubStillShot;
        }

        lastestHubShot = USE_SOTM ? hubSotmShot : hubStillShot;
        latestFeedShot = USE_SOTM ? feedSotmShot : feedStillShot;

        Logger.recordOutput("AimingManager/targetPosition", ObjectiveTarget.getFieldPosition());
        Logger.recordOutput("AimingManager/latestShot", latestShot);
        Logger.recordOutput("AimingManager/hubShotSotm", hubSotmShot);
        Logger.recordOutput("AimingManager/hubShotStill", hubStillShot);
        Logger.recordOutput("AimingManager/feedSotmShot", feedSotmShot);
        Logger.recordOutput("AimingManager/feedStillShot", feedStillShot);
    }
}
