package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterConstants {
    public static final int MAIN_FLY_ID = 1;
    public static final int FOLLOWER_FLY_ID = 2;
    public static final int HOOD_MOTOR_ID = 3;
    public static final int KICKER_ID = 4;

    

    public static final double HOOD_GEAR_RATIO = 52.0/1.0;  // one motor rotation for hood vortex is 1/52 of a full hood rotation
    public static final double FLYWHEEL_GEAR_RATIO = 1.0/1.25;
    public static final double KICKER_GEAR_RATIO = 1.0;

    public static final double HOOD_POS_FACTOR = (1.0/HOOD_GEAR_RATIO) * (2 * Math.PI);  // motor rotations -> hood rad
    public static final double HOOD_VEL_FACTOR = ((1.0/HOOD_GEAR_RATIO) * (2 * Math.PI)) / 60;  // motor rpm -> hood rad/sec

    public static final double FLYWHEEL_POS_FACTOR = (1.0/FLYWHEEL_GEAR_RATIO) * (2 * Math.PI);  // motor rotations -> hood rad
    public static final double FLYWHEEL_VEL_FACTOR = ((1.0/FLYWHEEL_GEAR_RATIO) * (2 * Math.PI)) / 60;  // motor rpm -> hood rad/sec
    
    public static final double KICKER_POS_FACTOR = (1.0/KICKER_GEAR_RATIO) * (2 * Math.PI);  // motor rotations -> hood rad
    public static final double KICKER_VEL_FACTOR = ((1.0/KICKER_GEAR_RATIO) * (2 * Math.PI)) / 60;  // motor rpm -> hood rad/sec



    public static final double[] FLYWHEEL_PID = {0, 0, 0};
    public static final SimpleMotorFeedforward FLYWHEEL_FEEDFORWARD = new SimpleMotorFeedforward(1,1,1);

    public static final double[] KICKER_PID = {0, 0, 0};
    public static final SimpleMotorFeedforward KICKER_FEEDFORWARD = new SimpleMotorFeedforward(1, 1, 1);

    public static final double[] HOOD_PID = {0, 0, 0};
    public static final ArmFeedforward HOOD_FEEDFORWARD = new ArmFeedforward(1, 1, 1, 1);

    public static final double HOOD_LENGTH_METERS = 0.2032;
    public static final double HOOD_MIN_ANGLE = 0;
    public static final double HOOD_MAX_ANGLE = 1.5708;
}
