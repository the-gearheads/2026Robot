package frc.robot.constants;

import static frc.robot.constants.ShooterConstants.KICKER_FEEDFORWARD;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final int MAIN_FLY_ID = 40;
    public static final int FOLLOWER_FLY_ID = 41;
    public static final int HOOD_MOTOR_ID = 42;
    public static final int KICKER_ID = 43;

    // https://gemini.google.com/share/e8d7da86ce5d for diameter math
    public static final double EFFECTIVE_FLYWHEEL_DIAMETER = Units.inchesToMeters(3.38);
    public static final double EFFECTIVE_KICKER_DIAMETER = Units.inchesToMeters(2.0625);
    public static final double KICKER_SURFACE_SPEED_RATIO = 0.7; // kicker surface speed is this percent of flywheel surface speed

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

    public static final double[] FLYWHEEL_PID = {0.001, 0, 0};
    public static final SimpleMotorFeedforward FLYWHEEL_FEEDFORWARD = new SimpleMotorFeedforward(0.088376, 0.01389, 0.00072178);

    public static final double[] KICKER_PID = {0.001, 0, 0};
    public static final SimpleMotorFeedforward KICKER_FEEDFORWARD = new SimpleMotorFeedforward(0.18189, 0.017501, 0.00038719);

    // public static final double[] HOOD_PID = {0.14218, 0, 0};
    public static final double[] HOOD_PID = {10, 0, 0.5};
    public static final ArmFeedforward HOOD_FEEDFORWARD = new ArmFeedforward(0.19535, 0.14086, 0.75006, 0.019405); 
    // public static final ArmFeedforward HOOD_FEEDFORWARD = new ArmFeedforward(0, 0, 0, 0); 
    public static final Constraints HOOD_CONSTRAINTS = new Constraints(
        Units.degreesToRadians(1500), // per second; max vel
        Units.degreesToRadians(1000)  //  per sec^2; max accel
    );

    // public static final ArmFeedforward HOOD_FEEDFORWARD = new ArmFeedforward(0, 0, 0, 0);  // 0.75006
    public static final Rotation2d HOOD_ANGLE_OFFSET = Rotation2d.fromRadians(-1.59);
    
    public static final double HOOD_LENGTH_METERS = Units.inchesToMeters(8.07); // placeholder; from pivot point to out edge, for sim
    public static final double HOOD_MIN_ANGLE = Units.degreesToRadians(0);  // hood 0 is 7.5 deg up
    public static final double HOOD_MAX_ANGLE = Units.degreesToRadians(47);

    public static final Rotation2d HOOD_ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.2);

    public static final double HOOD_MIN_SYSID_ANGLE = Units.degreesToRadians(5);  // more conservative so sysid doesn't break anything
    public static final double HOOD_MAX_SYSID_ANGLE = Units.degreesToRadians(45);

    public static final double FLYWHEEL_RADIUS = Units.inchesToMeters(3);  // i mean its not, but it is, but its not

    public static final double[] SHOOT_DISTANCES = {1, 2, 3};  // in meters
    public static final double[] SHOOT_ANGLES = {1, 2, 3};  // in Radians
    public static final double[] SHOOT_RPMS = {1, 2, 3};  // in Radians/Sec

    public static final double MAX_KICKER_SPEED = KICKER_FEEDFORWARD.maxAchievableVelocity(12, 0);
    public static final double MAX_EFFECTIVE_FLYWHEEL_SPEED = (MAX_KICKER_SPEED * EFFECTIVE_KICKER_DIAMETER) / (EFFECTIVE_FLYWHEEL_DIAMETER * KICKER_SURFACE_SPEED_RATIO);  // in order to maintain the surface speed ratio, the flywheel can't go faster than this speed or the kicker will be commanded to go faster than its max speed

    public static final Translation3d CENTER_BOT_TOSHOOT = new Translation3d(Units.inchesToMeters(-7.5572), Units.inchesToMeters(9.2307), Units.inchesToMeters(20.5469));

}
