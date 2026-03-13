package frc.robot.util;


import static frc.robot.constants.ShooterConstants.FLYWHEEL_TOLERANCE;
import static frc.robot.constants.ShooterConstants.HOOD_ANGLE_TOLERANCE;
import static frc.robot.constants.ShooterConstants.KICKER_TOLERANCE;
import static frc.robot.constants.SwerveConstants.YAW_ALIGN_TOLERANCE;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class ShooterCalculations {
    // position of the goal, so we know where to shoot
    // 3 lists: one with distance to the goal (based on localization)
    // one with  the rpm that works best
    // one the shooter angle that works best

    // polynomanialsplinefunction or linear interpolating tree map and feed in those tables
    // we actually might need 2 bc theres 2 diff things that change
    // then the interpolationg table will figure out the angle and rpm frmo any distance within the min and max we measured from

    // onec we can shoot from anywhere stationary

    // we need yaw to be like the robot swerve point towards goal 
    

    // TO SHOOT ON MOVE after this:
    // time of flight recursion:
    // basically, its fancy things oblarg and eeswhar frmo the frc discord tell us how to do
    // it gives us a new 'fake' location for the hub, so we aim and shoot as if we were aiming towards that
    // and if shoot towards that new one while moving, it will go in the real hub 
    
    static InterpolatingDoubleTreeMap shooterAngleFunction = initAngleMap();
    static InterpolatingDoubleTreeMap shooterVelFunction = initShooterVelMap();
    public static ArrayList<Double> HubDists = new ArrayList<>();
    public static ArrayList<Double> ShooterSpeeds = new ArrayList<>();
    public static ArrayList<Rotation2d> HoodAngles = new ArrayList<>();

   private static InterpolatingDoubleTreeMap initAngleMap() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();

        for(int i=0; i<ShooterConstants.SHOOT_DISTANCES.length; i++) {
            map.put(ShooterConstants.SHOOT_DISTANCES[i], ShooterConstants.SHOOT_ANGLES[i]);
        }

        return map;
    }
        
    private static InterpolatingDoubleTreeMap initShooterVelMap() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        
        for(int i=0; i<ShooterConstants.SHOOT_SPEEDS.length; i++) {
            map.put(ShooterConstants.SHOOT_DISTANCES[i], ShooterConstants.SHOOT_SPEEDS[i]);
        }

        return map;
    }


    /**
     * NOT FOR USE IN AUTO
     * @param swerve
     * @param hood
     * @param shooter
     * @return boolean if aligned with the current Objective, FEEDING or HUB, doesn't wait for 0 robot velocity
     */
    public static boolean isReadyToShoot(Swerve swerve, Hood hood, Shooter shooter) {
        Pose2d robotPose = swerve.getPose();
        boolean yawReady = MathUtil.isNear(robotPose.getRotation().getRadians(), getRobotYaw(robotPose).getRadians(), YAW_ALIGN_TOLERANCE.getRadians());
        boolean shooterReady = MathUtil.isNear(shooter.getFlywheelVelocityRadPerSec(), getShootVelocity(swerve),
                FLYWHEEL_TOLERANCE) &&
                MathUtil.isNear(shooter.getKickerVelocityRadPerSec(), getKickerSpeed(getShootVelocity(swerve)), KICKER_TOLERANCE);
        boolean hoodReady = MathUtil.isNear(hood.getAngle().getRadians(), getShootAngle(swerve).getRadians(), HOOD_ANGLE_TOLERANCE.getRadians());
        Logger.recordOutput("ShooterCalculations/isReady/yawReady", yawReady);
        Logger.recordOutput("ShooterCalculations/isReady/shooterReady", shooterReady);
        Logger.recordOutput("ShooterCalculations/isReady/hoodReady", hoodReady);
        return yawReady && hoodReady && shooterReady;
    }

    public static boolean autonShootReady(Swerve swerve, Hood hood, Shooter shooter) {
        Pose2d robotPose = swerve.getPose();
        boolean yawReady = MathUtil.isNear(robotPose.getRotation().getRadians(), getAutonYaw(swerve).getRadians(), YAW_ALIGN_TOLERANCE.getRadians());
        boolean shooterReady = MathUtil.isNear(shooter.getFlywheelVelocityRadPerSec(), getAutonVelocity(swerve),
                FLYWHEEL_TOLERANCE) &&
                MathUtil.isNear(shooter.getKickerVelocityRadPerSec(), getKickerSpeed(getAutonVelocity(swerve)), KICKER_TOLERANCE);
        boolean hoodReady = MathUtil.isNear(hood.getAngle().getRadians(), getAutonAngle(swerve).getRadians(), HOOD_ANGLE_TOLERANCE.getRadians());
        Logger.recordOutput("ShooterCalculations/isReady/yawReadyAuton", yawReady);
        Logger.recordOutput("ShooterCalculations/isReady/shooterReadyAuton", shooterReady);
        Logger.recordOutput("ShooterCalculations/isReady/hoodReadyAuton", hoodReady);

        return yawReady && hoodReady && shooterReady;
    }

    public static Rotation2d getAutonAngle(Swerve swerve) {
        double hubDistance = getHubDistance(swerve.getPose());
        Rotation2d hubAngle = Rotation2d.fromRadians(shooterAngleFunction.get(hubDistance));
        return hubAngle;
    }
    
    public static double getAutonVelocity(Swerve swerve) {
        Pose2d robotPose = swerve.getPose();
        double hubSpeed = shooterVelFunction.get(getHubDistance(robotPose));
        return hubSpeed;
    }

    public static Rotation2d getAutonYaw(Swerve swerve) {
        Pose2d robotPose = swerve.getPose();
        Rotation2d robotYaw = getRobotYaw(robotPose);
        return robotYaw;
    }
    public static Rotation2d getShootAngle(Swerve swerve) {
        Pose2d robotPose = swerve.getPose();
        Rectangle2d[] trenchRectangles = getTrenchAvoidanceRectanlges(robotPose, swerve.getFieldRelativeSpeeds());
        for (int i = 0; i < trenchRectangles.length; i++) {
            if (trenchRectangles[i].contains(robotPose.getTranslation())) {
                Logger.recordOutput("ShooterConstants/inTrench", true);
                return Rotation2d.fromRadians(ShooterConstants.HOOD_MIN_ANGLE);
            }
        }
        Logger.recordOutput("ShooterConstants/inTrench", false);

        double hubDistance = getHubDistance(robotPose);
        Rotation2d hubAngle = Rotation2d.fromRadians(shooterAngleFunction.get(hubDistance));
        if (ObjectiveTracker.getObjective(robotPose) == Objective.HUB) {
            return hubAngle;
        } else if (ObjectiveTracker.getObjective(robotPose) == Objective.FEED_LEFT
                || ObjectiveTracker.getObjective(robotPose) == Objective.FEED_RIGHT
                || ObjectiveTracker.getObjective(robotPose) == Objective.FEED_OVER) {
            double feedDistance = getFeedingDistance(robotPose);
            Logger.recordOutput("ShooterCalculations/FeedingDistance", feedDistance);
            Rotation2d feedingAngle = Rotation2d.fromRadians(shooterAngleFunction.get(feedDistance));
            return feedingAngle;
        } else {
            return hubAngle;
        }
    }
    
    public static double getShootVelocity(Swerve swerve) {
        Pose2d robotPose = swerve.getPose();
        double hubSpeed = shooterVelFunction.get(getHubDistance(robotPose));
        if (ObjectiveTracker.getObjective(robotPose) == Objective.HUB) {
            return hubSpeed;
        } else if (ObjectiveTracker.getObjective(robotPose) == Objective.FEED_LEFT
                || ObjectiveTracker.getObjective(robotPose) == Objective.FEED_RIGHT
                || ObjectiveTracker.getObjective(robotPose) == Objective.FEED_OVER) {
            double feedDistance = getFeedingDistance(robotPose);
            Logger.recordOutput("ShooterCalculations/FeedingDistance", feedDistance);
            double feedingSpeed = shooterVelFunction.get(feedDistance);
            return feedingSpeed;
        } else {
            return hubSpeed;
        }
    }

    public static Rotation2d getRobotYaw(Pose2d robotPose) {
        Translation2d hubAngle = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        Translation2d targetAngle = hubAngle;
        if (ObjectiveTracker.getObjective(robotPose) == Objective.HUB) {
            targetAngle = hubAngle;
        } else if (ObjectiveTracker.getObjective(robotPose) == Objective.FEED_LEFT) {
            targetAngle = Objective.FEED_LEFT.aimingLocation;
        } else if (ObjectiveTracker.getObjective(robotPose) == Objective.FEED_RIGHT) {
            targetAngle = Objective.FEED_RIGHT.aimingLocation;
        } else if (ObjectiveTracker.getObjective(robotPose) == Objective.FEED_OVER) {
            targetAngle = Objective.FEED_OVER.aimingLocation;
        } else {
            targetAngle = hubAngle;
        }
        Rotation2d angle = targetAngle.minus(getShooterPosition(robotPose).getTranslation()).getAngle();
        return angle;
    }

    // ------------------------

    private static double getFeedingDistance(Pose2d robotPose) {
        Translation2d feedPosition = ObjectiveTracker.getFeedingObjective(robotPose).aimingLocation; 
        return feedPosition.getDistance(getShooterPosition(robotPose).getTranslation());
    }
    
    private static double getHubDistance(Pose2d robotPose) { // TODO RENAME TO getObjectiveDistance
        Translation2d hubPosition = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        return hubPosition.getDistance(getShooterPosition(robotPose).getTranslation());
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

    public static Rectangle2d[] getTrenchAvoidanceRectanlges(Pose2d pose, ChassisSpeeds fieldRelativeRobotSpeed) {
        Translation2d[] hubSideCorners = {
            new Translation2d(4.63, 1.5),
            new Translation2d(4.63, 6.68),
            new Translation2d(11.918, 1.5),
            new Translation2d(11.918, 6.68)
        };

        Translation2d[] WallSideCorners = {  // 0.3 is a tiny bit of tolerance in case localization is outside of the field
            new Translation2d(4.63, 0 - 0.3),
            new Translation2d(4.63, FieldConstants.fieldWidth + 0.3),
            new Translation2d(11.918, 0 - 0.3),
            new Translation2d(11.918, FieldConstants.fieldWidth + 0.3)
        };

        double rectWidth = (fieldRelativeRobotSpeed.vxMetersPerSecond * 0.5) + Units.inchesToMeters(45);
        Rectangle2d[] scaryZones = new Rectangle2d[hubSideCorners.length];
        for(int i=0; i<hubSideCorners.length; i++) {
            Rectangle2d currentZone = new Rectangle2d(hubSideCorners[i].plus(new Translation2d(rectWidth, 0)), WallSideCorners[i].plus(new Translation2d(-rectWidth, 0)));
            Logger.recordOutput("ShooterCalculations/zoneCorners" + i, hubSideCorners[i].plus(new Translation2d(rectWidth, 0)), WallSideCorners[i].plus(new Translation2d(-rectWidth, 0)));
            scaryZones[i] = currentZone;
        }
        return scaryZones;
    }

    // see https://gemini.google.com/share/e8d7da86ce5d for explanation, returns motor velocity to get kicker to proportional speed as flywheel; keep in mind flywheel is geared up.
    public static double getKickerSpeed(double flywheelSpeed) {
        double kickerSpeed = flywheelSpeed * ShooterConstants.KICKER_SURFACE_SPEED_RATIO * (ShooterConstants.EFFECTIVE_FLYWHEEL_DIAMETER / ShooterConstants.EFFECTIVE_KICKER_DIAMETER);
        return kickerSpeed;
    }

    public static void log(Pose2d robotPose) {
        ObjectiveTracker.log(robotPose);
        Logger.recordOutput("ShooterCalculations/hubDistance", getHubDistance(robotPose));
    }
}
