package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    public static final int DEPLOY_ID = 30;
    public static final int INTAKE_ID = 31;
    public static final int DEPLOY_ENCODER_ID = 34;
    public static final int INTAKE_CURRENT_LIMIT = 60;
    public static final int DEPLOY_CURRENT_LIMIT = 40;

    public static final double[] DEPLOY_PID = {1.2, 0, 0};

    public static final float DEPLOY_POS_FACTOR = (float)(2 * Math.PI);
    public static final float DEPLOY_VEL_FACTOR = (float)((2 * Math.PI) / 60.0);  // why tf does it want a float TODO: complain to rev
    public static final float DEPLOY_OFFSET = (float)0.476;  // this must be in rotations

    public static final double INTAKE_GEAR_RATIO = 1;

    // public static final double DEPLOY_GEAR_RATIO = 60.0/1.0;  // placeholder
    // public static final double DEPLOY_POS_FACTOR = (1/DEPLOY_GEAR_RATIO);

    public static final double DEPLOY_LENGTH = Units.inchesToMeters(21);  // not exact

    public static final Rotation2d DEPLOY_MIN_ANGLE = Rotation2d.kZero;
    public static final Rotation2d DEPLOY_MAX_ANGLE = Rotation2d.fromRadians(0.230);

    public static final Rotation2d DEPLOY_MIN_SYSID_ANGLE = Rotation2d.fromDegrees(4);
    public static final Rotation2d DEPLOY_MAX_SYSID_ANGLE = Rotation2d.fromRadians(0.21);

    public static final Rotation2d DEPLOY_HOLD_ANGLE = Rotation2d.fromRadians(0.002);
    public static final Rotation2d DEPLOY_SHIMMY_ANGLE = Rotation2d.fromDegrees(30);
    public static final double DEPLOY_SHIMMY_TOLERANCE = 2.5;


}