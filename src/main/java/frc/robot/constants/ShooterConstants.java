package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final int MAIN_FLY_ID = 96;
    public static final int FOLLOWER_FLY_ID = 97;
    public static final int HOOD_MOTOR_ID = 98;
    public static final int KICKER_ID = 99;

    

    public static final double HOOD_GEAR_RATIO = 52.0/1.0;  // one motor rotation for hood vortex is 1/52 of a full hood rotation
    public static final double FLYWHEEL_GEAR_RATIO = 1.0/1.25;  // upduction, flywheel faster than motor
    public static final double KICKER_GEAR_RATIO = 1.0;

    // multiplying by these factors should convert from RPM or Rotations into the final unit on the final output shaft. Diving does opposite
    public static final double HOOD_POS_FACTOR = (1.0/HOOD_GEAR_RATIO) * (2 * Math.PI);  // motor rotations -> hood rad
    public static final double HOOD_VEL_FACTOR = ((1.0/HOOD_GEAR_RATIO) * (2 * Math.PI)) / 60;  // motor rpm -> hood rad/sec

    public static final double FLYWHEEL_POS_FACTOR = (1.0/FLYWHEEL_GEAR_RATIO) * (2 * Math.PI);  // motor rotations -> hood rad
    public static final double FLYWHEEL_VEL_FACTOR = ((1.0/FLYWHEEL_GEAR_RATIO) * (2 * Math.PI)) / 60;  // motor rpm -> hood rad/sec
    
    public static final double KICKER_POS_FACTOR = (1.0/KICKER_GEAR_RATIO) * (2 * Math.PI);  // motor rotations -> hood rad
    public static final double KICKER_VEL_FACTOR = ((1.0/KICKER_GEAR_RATIO) * (2 * Math.PI)) / 60;  // motor rpm -> hood rad/sec



    public static final double[] FLYWHEEL_PID = {0, 0, 0};
    public static final SimpleMotorFeedforward FLYWHEEL_FEEDFORWARD = new SimpleMotorFeedforward(0.045537, 0.0017932, 0.0001929);

    public static final double[] KICKER_PID = {0, 0, 0};
    public static final SimpleMotorFeedforward KICKER_FEEDFORWARD = new SimpleMotorFeedforward(0.045537, 0.0017932, 0.0001929);

    public static final double[] HOOD_PID = {0, 0, 0};
    public static final ArmFeedforward HOOD_FEEDFORWARD = new ArmFeedforward(0.2509, 0.099081, 5.5782, 0.28261);

    public static final double HOOD_LENGTH_METERS = Units.inchesToMeters(8.07); // placeholder; from pivot point to out edge, for sim
    public static final double HOOD_MIN_ANGLE = Units.degreesToRadians(0);
    public static final double HOOD_MAX_ANGLE = Units.degreesToRadians(50);

    public static final double HOOD_MIN_SYSID_ANGLE = Units.degreesToRadians(5);  // more conservative so sysid doesn't break anything
    public static final double HOOD_MAX_SYSID_ANGLE = Units.degreesToRadians(45);

    public static final double FLYWHEEL_RADIUS = Units.inchesToMeters(3.5);
}
