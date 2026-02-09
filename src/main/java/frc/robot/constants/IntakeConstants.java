package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    public static final int DEPLOY_ID = 93;
    public static final int INTAKE_ID = 92;

    public static final int INTAKE_CURRENT_LIMIT = 60;
    public static final int DEPLOY_CURRENT_LIMIT = 60;

    public static final double[] DEPLOY_PID = {1, 0, 0};

    public static final double INTAKE_GEAR_RATIO = 1;

    // public static final double DEPLOY_GEAR_RATIO = 60.0/1.0;  // placeholder
    // public static final double DEPLOY_POS_FACTOR = (1/DEPLOY_GEAR_RATIO);

    public static final double DEPLOY_LENGTH = Units.inchesToMeters(21);  // not exact

    public static final Rotation2d DEPLOY_MIN_ANGLE = Rotation2d.kZero;
    public static final Rotation2d DEPLOY_MAX_ANGLE = Rotation2d.fromDegrees(50);
}