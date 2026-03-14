package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.MiscConstants;

public enum Objective {
    HUB(AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())),
    FEED_OVER(MiscConstants.OVER_FEEDING_LOCATION),
    FEED_LEFT(AllianceFlipUtil.apply(MiscConstants.LEFT_FEEDING_LOCATION)),
    FEED_RIGHT(AllianceFlipUtil.apply(MiscConstants.RIGHT_FEEDING_LOCATION));

    public final Translation2d aimingLocation;
    private Objective(Translation2d aimingLocation) {
        this.aimingLocation = aimingLocation;
    }

    public Objective getFeedingObjective(Pose2d robotPose) {
        if(robotPose.getY() < FieldConstants.fieldWidth/2.0) {
            return Objective.FEED_RIGHT;
        } else {
            return Objective.FEED_LEFT;
        }
    } 

}
