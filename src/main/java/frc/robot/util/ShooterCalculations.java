package frc.robot.util;


import static frc.robot.constants.ShooterConstants.FLYWHEEL_TOLERANCE;
import static frc.robot.constants.ShooterConstants.HOOD_ANGLE_TOLERANCE;
import static frc.robot.constants.ShooterConstants.HUB_ANGLE_ADJUSTMENT;
import static frc.robot.constants.ShooterConstants.KICKER_TOLERANCE;
import static frc.robot.constants.ShooterConstants.SHOOTER_RPM_ADJUSTMENT;
import static frc.robot.constants.SwerveConstants.YAW_ALIGN_TOLERANCE;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    static InterpolatingDoubleTreeMap shooterToFFunction = initShooterToFMap();
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

    private static InterpolatingDoubleTreeMap initShooterToFMap() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        
        for(int i=0; i<ShooterConstants.SHOOT_TOFS.length; i++) {
            map.put(ShooterConstants.SHOOT_DISTANCES[i], ShooterConstants.SHOOT_TOFS[i]);
        }

        return map;
    }

    //  public record ShotData(double Speed, Rotation2d Angle, Translation2d target) {
    //     public ShotData(double Speed, Rotation2d Angle, Translation2d target) {
    //          public ShotData(double Speed, Rotation2d Angle, Translation2d target) {
    //         this(Speed, Angle, target);
    //     }

    //     public ShotData(AngularVelocity exitVelocity, Angle hoodAngle) {
    //         this(exitVelocity, hoodAngle, FieldConstants.HUB_BLUE);
    //     }

    //     public ShotData(double exitVelocity, double hoodAngle) {
    //         this(exitVelocity, hoodAngle, FieldConstants.HUB_BLUE);
    //     }

    //     public LinearVelocity getLinearExitVelocity() {
    //         return angularToLinearVelocity(RadiansPerSecond.of(this.exitVelocity), FLYWHEEL_RADIUS);
    //     }

    //     public AngularVelocity getAngularExitVelocity() {
    //         return RadiansPerSecond.of(this.exitVelocity);
    //     }

    //     public Angle getHoodAngle() {
    //         return Radians.of(this.hoodAngle);
    //     }

    //     public Translation3d getTarget() {
    //         return this.target;
    //     }

    //     public static ShotData interpolate(ShotData start, ShotData end, double t) {
    //         return new ShotData(
    //                 MathUtil.interpolate(start.exitVelocity, end.exitVelocity, t),
    //                 MathUtil.interpolate(start.hoodAngle, end.hoodAngle, t),
    //                 end.target);
    //     }
    //     }



    /**
     * NOT FOR USE IN AUTO
     * @param swerve
     * @param hood
     * @param shooter
     * @return boolean if aligned with the current Objective, FEEDING or HUB, doesn't wait for 0 robot velocity
     */
    public static boolean isReadyToShoot(Swerve swerve, Hood hood, Shooter shooter) {
        Pose2d robotPose = swerve.getPose();
        boolean yawReady = MathUtil.isNear(robotPose.getRotation().getRadians(), getObjectiveRobotYaw(robotPose).getRadians(), YAW_ALIGN_TOLERANCE.getRadians());
        boolean shooterReady = MathUtil.isNear(shooter.getFlywheelVelocityRadPerSec(), getObjectiveShootVelocity(swerve),
                FLYWHEEL_TOLERANCE) &&
                MathUtil.isNear(shooter.getKickerVelocityRadPerSec(), getKickerSpeed(getObjectiveShootVelocity(swerve)), KICKER_TOLERANCE);
        boolean hoodReady = MathUtil.isNear(hood.getAngle().getRadians(), getObjectiveHoodAngle(swerve).getRadians(), HOOD_ANGLE_TOLERANCE.getRadians());
        Logger.recordOutput("ShooterCalculations/isReady/yawReady", yawReady);
        Logger.recordOutput("ShooterCalculations/isReady/shooterReady", shooterReady);
        Logger.recordOutput("ShooterCalculations/isReady/hoodReady", hoodReady);
        return yawReady && hoodReady && shooterReady;
    }

    public static boolean hubShootReady(Swerve swerve, Hood hood, Shooter shooter) {
        Pose2d robotPose = swerve.getPose();
        boolean yawReady = MathUtil.isNear(robotPose.getRotation().getRadians(), getHubYaw(swerve).getRadians(), YAW_ALIGN_TOLERANCE.getRadians());
        boolean shooterReady = MathUtil.isNear(shooter.getFlywheelVelocityRadPerSec(), getHubVelocity(swerve),
                FLYWHEEL_TOLERANCE) &&
                MathUtil.isNear(shooter.getKickerVelocityRadPerSec(), getKickerSpeed(getHubVelocity(swerve)), KICKER_TOLERANCE);
        boolean hoodReady = MathUtil.isNear(hood.getAngle().getRadians(), getHubHoodAngle(swerve).getRadians(), HOOD_ANGLE_TOLERANCE.getRadians());
        Logger.recordOutput("ShooterCalculations/isReady/yawReadyHub", yawReady);
        Logger.recordOutput("ShooterCalculations/isReady/shooterReadyHub", shooterReady);
        Logger.recordOutput("ShooterCalculations/isReady/hoodReadyHub", hoodReady);

        return yawReady && hoodReady && shooterReady;
    }

    public static Rotation2d getHubHoodAngle(Swerve swerve) {
        double hubDistance = getHubDistance(swerve.getPose());
        Rotation2d hubAngle = Rotation2d.fromRadians(shooterAngleFunction.get(hubDistance)).plus(HUB_ANGLE_ADJUSTMENT);
        Logger.recordOutput("ShooterCalculations/AutonAngle", hubAngle);
        return hubAngle;
    }
    
    public static double getHubVelocity(Swerve swerve) {
        Pose2d robotPose = swerve.getPose();
        double hubSpeed = shooterVelFunction.get(getHubDistance(robotPose));
        Logger.recordOutput("ShooterCalculations/AutonVelocity", hubSpeed);
        return hubSpeed;
    }

    public static Rotation2d getHubYaw(Swerve swerve) {
        Translation2d hubAngle = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        Rotation2d angle = hubAngle.minus(getShooterPosition(swerve.getPose()).getTranslation()).getAngle();
        Logger.recordOutput("ShooterCalculations/AutonYaw", angle);
        return angle;
    }

    public static Rotation2d getFeederHoodAngle(Swerve swerve) {
        double feedDistance = getFeedingDistance(swerve.getPose());
        Logger.recordOutput("ShooterCalculations/FeedingDistance", feedDistance);
        Rotation2d feedingAngle = Rotation2d.fromRadians(shooterAngleFunction.get(feedDistance));
        Logger.recordOutput("ShooterCalculations/FeederHoodAngle", feedingAngle);
        return feedingAngle;
    }
    
    public static double getFeedVelocity(Swerve swerve) {
        double feedDistance = getFeedingDistance(swerve.getPose());
        Logger.recordOutput("ShooterCalculations/FeedingDistance", feedDistance);
        double feedingSpeed = shooterVelFunction.get(feedDistance);
        Logger.recordOutput("ShooterCalculations/FeedShootVel", feedingSpeed);
        return feedingSpeed;
    }

    public static Rotation2d getFeederYaw(Swerve swerve) {
        Pose2d robotPose = swerve.getPose();
        Translation2d targetAngle = Objective.FEED_LEFT.aimingLocation;
        if (ObjectiveTracker.getObjective(robotPose) == Objective.FEED_LEFT) {
            targetAngle = Objective.FEED_LEFT.aimingLocation;
        } else if (ObjectiveTracker.getObjective(robotPose) == Objective.FEED_RIGHT) {
            targetAngle = Objective.FEED_RIGHT.aimingLocation;
        } else if (ObjectiveTracker.getObjective(robotPose) == Objective.FEED_OVER) {
            targetAngle = Objective.FEED_OVER.aimingLocation;
        }

        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                targetAngle = AllianceFlipUtil.apply(targetAngle);
            }
        }
        Rotation2d angle = targetAngle.minus(getShooterPosition(robotPose).getTranslation()).getAngle();
        Logger.recordOutput("ShooterCalculations/FeederRobotYaw", angle);
        return angle;
    }

    public static Rotation2d getObjectiveHoodAngle(Swerve swerve) {
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
        Rotation2d hubAngle = Rotation2d.fromRadians(shooterAngleFunction.get(hubDistance)).plus(HUB_ANGLE_ADJUSTMENT);
        if (ObjectiveTracker.getObjective(robotPose) == Objective.HUB) {
            return hubAngle;
        } else if (ObjectiveTracker.getObjective(robotPose) == Objective.FEED_LEFT
                || ObjectiveTracker.getObjective(robotPose) == Objective.FEED_RIGHT
                || ObjectiveTracker.getObjective(robotPose) == Objective.FEED_OVER) {
            double feedDistance = getFeedingDistance(robotPose);
            Logger.recordOutput("ShooterCalculations/FeedingDistance", feedDistance);
            Rotation2d feedingAngle = Rotation2d.fromRadians(shooterAngleFunction.get(feedDistance));
            Logger.recordOutput("ShooterCalculations/ObjectiveHoodAngle", feedingAngle);
            return feedingAngle;
        } else {
            Logger.recordOutput("ShooterCalculations/ObjectiveHoodAngle", hubAngle);
            return hubAngle;
        }
    }
    
    public static double getObjectiveShootVelocity(Swerve swerve) {
        Pose2d robotPose = swerve.getPose();
        double hubSpeed = (shooterVelFunction.get(getHubDistance(robotPose)) + SHOOTER_RPM_ADJUSTMENT);
        if (ObjectiveTracker.getObjective(robotPose) == Objective.HUB) {
            return hubSpeed;
        } else if (ObjectiveTracker.getObjective(robotPose) == Objective.FEED_LEFT
                || ObjectiveTracker.getObjective(robotPose) == Objective.FEED_RIGHT
                || ObjectiveTracker.getObjective(robotPose) == Objective.FEED_OVER) {
            double feedDistance = getFeedingDistance(robotPose);
            Logger.recordOutput("ShooterCalculations/FeedingDistance", feedDistance);
            double feedingSpeed = shooterVelFunction.get(feedDistance);
            Logger.recordOutput("ShooterCalculations/ObjectiveShooterVel", feedingSpeed);
            return feedingSpeed;
        } else {
            Logger.recordOutput("ShooterCalculations/ObjectiveShooterVel", hubSpeed);
            return hubSpeed;
        }
    }

    public static Rotation2d getObjectiveRobotYaw(Pose2d robotPose) {
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
        Logger.recordOutput("ShooterCalculations/OjectiveRobotYaw", angle);
        return angle;
    }

    // ------------------------

    private static double getFeedingDistance(Pose2d robotPose) {
        Translation2d feedPosition = ObjectiveTracker.getFeedingObjective(robotPose).aimingLocation; 
        return feedPosition.getDistance(getShooterPosition(robotPose).getTranslation());
    }
    
    public static double getHubDistance(Pose2d robotPose) { // fuck that other comment its hub distance
        Translation2d hubPosition = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        return hubPosition.getDistance(getShooterPosition(robotPose).getTranslation());
    }

    // TIME OF FLIGHT AND STOM CODE HERE ----
    private static Translation2d getAdjustedHubPose(Swerve swerve) {
        Translation2d hubPosition = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

        // uhh hub speed is like. getHubDistance/ToF
    }

    private static double getHubTimeOfFlight(double distanceToHub) {
        return shooterToFFunction.get(distanceToHub).doubleValue();
    }

    public static double getDistanceToTarget(Pose2d robotPose, Translation2d target) {
        return target.getDistance(getShooterPosition(robotPose).getTranslation());
    }

    // public static ShotData iterativeMovingShot(
    //         Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
    //     // Perform initial estimation (assuming unmoving robot) to get time of flight estimate
    //     ShotData shot = calculateHubShot(robot, target, target);
    //     double distance = getDistanceToTarget(robot, target.toTranslation2d());
    //     double timeOfFlight = getHubTimeOfFlight(getDistanceToTarget(robot, target.toTranslation2d()));
    //     Translation3d predictedTarget = target;

    //     // Iterate the process, getting better time of flight estimations and updating the predicted target accordingly
    //     for (int i = 0; i < iterations; i++) {
    //         predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
    //         timeOfFlight = getHubTimeOfFlight(getDistanceToTarget(robot, predictedTarget.toTranslation2d()));
    //     }
    //     return shot;
    // }
    
    // Move a target a set time in the future along a velocity defined by fieldSpeeds
    public static Translation3d predictTargetPos(Translation3d target, ChassisSpeeds fieldSpeeds, double tofSeconds) {
        double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * tofSeconds;
        double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * tofSeconds;

        return new Translation3d(predictedX, predictedY, target.getZ());
    }
    // END SOTM CODE -----------

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
