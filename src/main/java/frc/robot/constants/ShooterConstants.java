package frc.robot.constants;

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
    public static final SimpleMotorFeedforward FLYWHEEL_FEEDFORWARD = new SimpleMotorFeedforward(0.088376, 0.01352, 0.00072178);  // 0.088376, 0.01339, 0.00072178

    public static final double[] KICKER_PID = {0.001, 0, 0};
    public static final SimpleMotorFeedforward KICKER_FEEDFORWARD = new SimpleMotorFeedforward(0.18189, 0.017501, 0.00038719);

    public static final double[] HOOD_PID = {12.0, 0, 0.25};
    public static final double HOOD_I_ZONE = Units.degreesToRadians(1.5);
    public static final double HOOD_MAX_I_ACCUM = 0.5;  // volts max from integrator
    // public static final double HOOD_UP_KS = 0.255;  // volts to make a continous motion without stopping
    public static final double HOOD_UP_KS = 0.2;  // volts to make a continous motion without stopping
    public static final double HOOD_DOWN_KS = -0.18;

    public static final SimpleMotorFeedforward HOOD_FEEDFORWARD = new SimpleMotorFeedforward(0, 0.75, 0.022);
    public static final Constraints HOOD_CONSTRAINTS = new Constraints(
        Units.degreesToRadians(2000), // per second; max vel
        Units.degreesToRadians(3000)  //  per sec^2; max accel
    );

    // public static final Rotation2d HOOD_ANGLE_OFFSET = Rotation2d.fromRadians(-1.59);
    
    public static final double HOOD_LENGTH_METERS = Units.inchesToMeters(8.07); // placeholder; from pivot point to out edge, for sim
    public static final Rotation2d HOOD_MIN_ANGLE = Rotation2d.fromDegrees(0);  // hood 0 is 7.5 deg up
    public static final Rotation2d HOOD_MAX_ANGLE = Rotation2d.fromDegrees(47);  

    public static final Rotation2d HOOD_ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.1);
    public static final Rotation2d HOOD_MOVING_TOLERANCE = Rotation2d.fromDegrees(0.5);

    public static final double HOOD_MIN_SYSID_ANGLE = Units.degreesToRadians(5);  // more conservative so sysid doesn't break anything
    public static final double HOOD_MAX_SYSID_ANGLE = Units.degreesToRadians(45);

    public static final double FLYWHEEL_RADIUS = Units.inchesToMeters(3);  // i mean its not, but it is, but its not

    // 24 data points
    public static final double[] SHOOT_DISTANCES = { 1.227,   1.392,   1.564,   1.688,   1.858,   2.008,   2.215,   2.420,   2.621,   2.819,   2.979,   3.191,   3.404,   3.611,   3.859,   4.062,   4.304,   4.498,   4.547,   4.762,   5.018,   5.247,   5.447,   5.681,   5.865   }; // Meters
    public static final double[] SHOOT_ANGLES    = { 0.016,   0.034,   0.055,   0.077,   0.104,   0.121,   0.156,   0.165,   0.184,   0.201,   0.208,   0.225,   0.237,   0.252,   0.269,   0.296,   0.312,   0.318,   0.314,   0.313,   0.314,   0.330,   0.348,   0.347,   0.356   }; // Radians
    public static final double[] SHOOT_SPEEDS    = { 231.467, 232.617, 231.850, 234.425, 234.480, 230.645, 232.782, 244.013, 252.669, 256.339, 265.324, 270.638, 268.611, 275.404, 280.773, 280.719, 289.320, 306.522, 315.891, 316.055, 325.642, 332.983, 338.955, 347.666, 362.458 }; // Rad/Sec
    public static final double[] SHOOT_TOFS   = { 1.13888, 1.18095, 1.17708, 1.18750, 1.15555, 1.18541, 1.214583333, 1.13500, 1.133333333, 1.221527778, 1.21333, 1.25000, 1.27569, 1.26500, 1.31166, 1.29583, 1.24861, 1.23500, 1.23611, 1.25750, 1.31833, 1.37777, 1.37833, 1.40972, 1.43263 };
    //public static final double[] SHOOT_TOFS      = { 1.13888, 1.18095, 1.17708, 1.18750, 1.15555, 1.18541, 1.15   , 1.13500, 1.15,    1.18,    1.21333, 1.25000, 1.27569, 1.26500, 1.31166, 1.29583, 1.24861, 1.23500, 1.23611, 1.25750, 1.31833, 1.37777, 1.37833, 1.40972, 1.43263 };

    // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/linear-drag.html
    public static final double DRAG_CONSTANT = 0.2;
    public static final double LATENCY_COMPENSATION = 0.10;  // seconds; time accounting for latency for everything other than just real ball ToF 

    public static final double MAX_KICKER_SPEED = KICKER_FEEDFORWARD.maxAchievableVelocity(12, 0);
    public static final double MAX_EFFECTIVE_FLYWHEEL_SPEED = (MAX_KICKER_SPEED * EFFECTIVE_KICKER_DIAMETER) / (EFFECTIVE_FLYWHEEL_DIAMETER * KICKER_SURFACE_SPEED_RATIO);  // in order to maintain the surface speed ratio, the flywheel can't go faster than this speed or the kicker will be commanded to go faster than its max speed

    public static final Translation3d CENTER_BOT_TOSHOOT = new Translation3d(Units.inchesToMeters(-7.5572), Units.inchesToMeters(-9.2307), Units.inchesToMeters(20.5469));

    public static final double FLYWHEEL_TOLERANCE = Units.rotationsPerMinuteToRadiansPerSecond(100);
    public static final double KICKER_TOLERANCE = Units.rotationsPerMinuteToRadiansPerSecond(100);

    public static final double DEPOT_TRENCH_SHOOT_VELOCITY = 262.42;  // TODO: fix these
    public static final Rotation2d DEPOT_TRENCH_SHOOT_ANGLE = new Rotation2d(0.258); 

    public static final double HP_TRENCH_SHOOT_VELOCITY = 262.42;
    public static final Rotation2d HP_TRENCH_SHOOT_ANGLE = new Rotation2d(0.258); 

    public static Rotation2d HOOD_ANGLE_ADJUSTMENT = Rotation2d.fromDegrees(0);
    public static double SHOOTER_VEL_ADJUSTMENT = Units.rotationsPerMinuteToRadiansPerSecond(0);

}
