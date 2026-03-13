package frc.robot.util;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;

public class ObjectiveTracker {
    // takes in swerve, looks at hub status and robot position and velocity, maybe also considers Time of Flight for the balls
    // from there has a function to return what Objective we are aiming towards
    @AutoLogOutput
    public static boolean inAllianceZone(Pose2d robotPose) {
        double allianceZoneLine = AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone);
        Optional<Alliance> alliance = DriverStation.getAlliance();
    
        Rectangle2d blueAllianceZone = new Rectangle2d(new Translation2d(-0.1, -0.1), new Translation2d(allianceZoneLine+0.1, FieldConstants.fieldWidth));
        Rectangle2d redAllianceZone = new Rectangle2d(new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth), new Translation2d(allianceZoneLine, -0.1));
        if (alliance.isEmpty()) {
            return blueAllianceZone.contains(robotPose.getTranslation()) || redAllianceZone.contains(robotPose.getTranslation());
        } else if (alliance.get() == Alliance.Blue) {
            return blueAllianceZone.contains(robotPose.getTranslation());
        } else {
            return redAllianceZone.contains(robotPose.getTranslation());
        }
    }

    @AutoLogOutput
    public static Objective getFeedingObjective(Pose2d robotPose) {
        if(AllianceFlipUtil.applyY(robotPose.getY()) < FieldConstants.fieldWidth/2.0) {
            return Objective.FEED_RIGHT;
        } else {
            return Objective.FEED_LEFT;
        }
    }

    @AutoLogOutput
    public static Objective getObjective(Pose2d robotPose) {
        if (HubTracker.isAllianceHubActive() && inAllianceZone(robotPose)) {
            return Objective.HUB;
        } else {
            return getFeedingObjective(robotPose);
        }
    }

    public static void log(Pose2d robotPose) {
        Logger.recordOutput("ObjectiveTracker/inAllianceZone", inAllianceZone(robotPose));
        Logger.recordOutput("ObjectiveTracker/ActiveAlliance", HubTracker.getActiveAlliance());
        Logger.recordOutput("ObjectiveTracker/Objective", getObjective(robotPose));
        Logger.recordOutput("ObjectiveTracker/FeedingObjective", getFeedingObjective(robotPose));
    }
}
