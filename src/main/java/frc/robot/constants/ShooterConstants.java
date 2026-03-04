package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final int MAIN_FLY_ID = 40;
    public static final int FOLLOWER_FLY_ID = 41;
    public static final int HOOD_MOTOR_ID = 42;
    public static final int KICKER_ID = 43;

    

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



    public static final double[] FLYWHEEL_PID = {0.0046686, 0, 0};
    public static final SimpleMotorFeedforward FLYWHEEL_FEEDFORWARD = new SimpleMotorFeedforward(0.088376, 0.013468, 0.00072178);

    public static final double[] KICKER_PID = {0.0010106, 0, 0};
    public static final SimpleMotorFeedforward KICKER_FEEDFORWARD = new SimpleMotorFeedforward(0.18189, 0.017894, 0.00038719);

    public static final double[] HOOD_PID = {0.040053, 0, 0};
    public static final ArmFeedforward HOOD_FEEDFORWARD = new ArmFeedforward(0.19624, 0.050267, 0.77833, 0.019925);

    public static final double HOOD_LENGTH_METERS = Units.inchesToMeters(8.07); // placeholder; from pivot point to out edge, for sim
    public static final double HOOD_MIN_ANGLE = Units.degreesToRadians(0);  // hood 0 is 7.5 deg up
    public static final double HOOD_MAX_ANGLE = Units.degreesToRadians(52);

    public static final double HOOD_MIN_SYSID_ANGLE = Units.degreesToRadians(5);  // more conservative so sysid doesn't break anything
    public static final double HOOD_MAX_SYSID_ANGLE = Units.degreesToRadians(45);

    public static final double FLYWHEEL_RADIUS = Units.inchesToMeters(3);  // i mean its not, but it is, but its not

    public static final double[] SHOOT_DISTANCES = {1, 2, 3};  // in meters
    public static final double[] SHOOT_ANGLES = {1, 2, 3};  // in Radians
    public static final double[] SHOOT_RPMS = {1, 2, 3};  // in Radians/Sec

    public static final Translation3d CENTER_BOT_TOSHOOT = new Translation3d(Units.inchesToMeters(-7.5572), Units.inchesToMeters(9.2307), Units.inchesToMeters(20.5469));

}
