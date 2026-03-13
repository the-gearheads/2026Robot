package frc.robot.util;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;

public class ObjectiveTracker {
    // takes in swerve, looks at hub status and robot position and velocity, maybe also considers Time of Flight for the balls
    // from there has a function to return what Objective we are aiming towards
    @AutoLogOutput
    public static boolean inAllianceZone(Pose2d robotPose) {
        double allianceZoneLine = AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone);
        Rectangle2d allianceZone = new Rectangle2d(AllianceFlipUtil.apply(new Translation2d(-0.1, -0.1)), AllianceFlipUtil.apply(new Translation2d(allianceZoneLine+0.1, FieldConstants.fieldWidth)));
        return allianceZone.contains(robotPose.getTranslation());
    }

    @AutoLogOutput
    public static Objective getFeedingObjective(Pose2d robotPose) {
        if(robotPose.getY() < FieldConstants.fieldWidth/2.0) {
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

}
