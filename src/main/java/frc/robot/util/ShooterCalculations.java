package frc.robot.util;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
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
    
        static InterpolatingDoubleTreeMap shooterAngleFunction = createAngleMap();
        static InterpolatingDoubleTreeMap shooterRPMFunction = createRPMFunction();
    
        static InterpolatingDoubleTreeMap createAngleMap() {
            InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
    
            for(int i=0; i<ShooterConstants.SHOOT_DISTANCES.length; i++) {
                map.put(ShooterConstants.SHOOT_DISTANCES[i], ShooterConstants.SHOOT_ANGLES[i]);
            }
    
            return map;
        }
    
        
        static InterpolatingDoubleTreeMap createRPMFunction() {
            InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
            
            for(int i=0; i<ShooterConstants.SHOOT_RPMS.length; i++) {
                map.put(ShooterConstants.SHOOT_DISTANCES[i], ShooterConstants.SHOOT_RPMS[i]);
            }
    
            return map;
        }
    
    
        public static double getHubDistance(Pose2d robotPose) {
            Translation2d hubPosition = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
            return hubPosition.getDistance(getShooterPosition(robotPose).getTranslation());
        } 
    
        public static Rotation2d getRobotYaw(Translation2d robotPose){
            Translation2d targetAngle = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    
            Rotation2d angle = targetAngle.minus(robotPose).getAngle();
            return angle;
        }
        
        public static Rotation2d getHoodAngle(Swerve swerve) {
            Rotation2d hubAngle = Rotation2d.fromRadians(shooterAngleFunction.get(getHubDistance(getShooterPosition(swerve.getPose()))));
            return hubAngle;
        }
    
        public static double getShooterVelocity(Pose2d robotPose) {
            return shooterRPMFunction.get(getHubDistance(getShooterPosition(robotPose)));
        }
    
        public static Pose2d getShooterPosition(Pose2d robotPose) {
            Translation2d shooterDistance = ShooterConstants.CENTER_BOT_TOSHOOT.toTranslation2d();
            
            Pose2d shooterPose = robotPose.plus(new Transform2d(shooterDistance, new Rotation2d(0)));

            return  shooterPose;
        }

        private static Rectangle2d[] getTrenchAvoidanceRectanlges(Pose2d pose, ChassisSpeeds robotSpeed) {
            Translation2d[] hubSideCorners = {
                new Translation2d(4.63, 1.5),
                new Translation2d(4.63, 6.68),
                new Translation2d(11.918, 1.5),
                new Translation2d(11.918, 6.68)
            };

            Translation2d[] WallSideCorners = {
                new Translation2d(4.63, 0),
                new Translation2d(4.63, FieldConstants.fieldWidth),
                new Translation2d(11.918, 0),
                new Translation2d(11.918, FieldConstants.fieldWidth)
            };

            double rectWidth = robotSpeed.vxMetersPerSecond * 0.5;
            Rectangle2d[] scaryZones = {};
            for(int i=0; i<hubSideCorners.length; i++) {
                Rectangle2d currentZone = new Rectangle2d(hubSideCorners[i].plus(new Translation2d(rectWidth, 0)), WallSideCorners[i].plus(new Translation2d(-rectWidth, 0)));
                scaryZones[i] = currentZone;
            }
            

            Logger.recordOutput("ShooterCalculations/scaryZones", scaryZones);
            return scaryZones;
        }
}
