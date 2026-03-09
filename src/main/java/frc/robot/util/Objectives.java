package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.MiscConstants;

public enum Objectives {
    HUB(AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())),
    FEED_OVER(AllianceFlipUtil.apply(MiscConstants.OVER_FEEDING_LOCATION)),
    FEED_LEFT(AllianceFlipUtil.apply(MiscConstants.LEFT_FEEDING_LOCATION)),
    FEED_RIGHT(AllianceFlipUtil.apply(MiscConstants.RIGHT_FEEDING_LOCATION));

    public final Translation2d aimingLocation;
    private Objectives(Translation2d aimingLocation) {
        this.aimingLocation = aimingLocation;
    }

}
