package frc.robot.constants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class SwerveConstants {
    public static final int[][] MOTOR_IDS = {
      {6, 26}, // FL
      {8, 28}, // FR
      {5, 22}, // BL
      {7, 24}  // BR
    };

    public static final int BORON_ID = 1;
    
    public static final Translation2d[] WHEEL_POSITIONS = {
      new Translation2d(0.3556, 0.31115),
      new Translation2d(0.3556, -0.31115),
      new Translation2d(-0.3556, 0.31115),
      new Translation2d(-0.3556, -0.31115)
    };
  
    public static final double WHEEL_BASE = WHEEL_POSITIONS[0].getX() - WHEEL_POSITIONS[2].getX();
    public static final double TRACK_WIDTH = WHEEL_POSITIONS[0].getY() - WHEEL_POSITIONS[1].getY();
    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);

    public static final double[] WHEEL_OFFSETS = {270, 180, 180, 90};
    public static final boolean[] IS_INVERTED = {true, false, true, false};

    public static final double WHEEL_RADIUS = 1.4609 / 100.0 * 2.54;  // derived from wheel radius characterizaiton
    public static final double WHEEL_DIAMETER = WHEEL_RADIUS * 2.0;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    public static final double DRIVE_PINION_TOOTH_COUNT = 12;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double DRIVE_RATIO = (45.0 * 22) / (DRIVE_PINION_TOOTH_COUNT * 15);

    public static final double ODOMETRY_FREQUENCY = 50;

    // Throughbore encoder is directly on the output steer shaft
    public static final double STEER_RATIO = 1;

    public static final double DRIVE_POS_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_RATIO; // rotations -> gear ratio adjusted rotations -> meters
    public static final double DRIVE_VEL_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_RATIO / 60.0; // rpm -> gear ratio adjusted rpm -> meters/min -> meters/sec 

    public static final double DRIVE_FREE_RPM = 6784;
    public static final double DRIVE_FREE_SPD = DRIVE_FREE_RPM * DRIVE_VEL_FACTOR; // Convert max neo free speed to max free wheel speed

    public static final double STEER_POS_FACTOR = 2 * Math.PI; // rotations -> radians
    public static final double STEER_VEL_FACTOR = (2 * Math.PI) / 60.0; // rpm -> rad/sec

    public static final double[] STEER_PIDF = {1, 0, 0, 0}; // apparently just a P value of 1 worked for us??? i wanna test that a bit more throughly. Update: we tested, 1 is goated
    public static final double[] DRIVE_PID = {3.6, 0, 0};
    public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.186587, 2.47975, 0.29612);

    public static final int DRIVE_CURRENT_LIMIT = 60;
    public static final int STEER_CURRENT_LIMIT = 20;
    
    public static final double MAX_MOD_SPEED = DRIVE_FREE_SPD;  // m/s, placeholders
    public static final double MAX_ROBOT_TRANS_SPEED = DRIVE_FEEDFORWARD.maxAchievableVelocity(12, 0.1); // 4.75 m/s
    public static final double MAX_ROBOT_ACCEL = DRIVE_FEEDFORWARD.maxAchievableAcceleration(12, MAX_ROBOT_TRANS_SPEED-1); // 8.47 m/s^2, 1 is a fudge factor
    public static final double MAX_MOD_STEER_VEL = Units.degreesToRadians(1040); // I think? Going from 0-90 went at ~200deg/s
    
    public static final double MAX_ROBOT_ROT_SPEED = MAX_ROBOT_TRANS_SPEED / 0.4585738763; // rad/s, 0.45 is radius of robot, spd/r is rad/s

    public static final double HEADING_CONTROLLER_TOLERANCE = 0.005;  // rad

    public static double[] XY_PATH_FOLLOWING_PID = {1.5, 0, 0.0};
    public static double[] ROT_PATH_FOLLOWING_PID = {0.8, 0, 0};

    public static double[] DRIVE_CONTROLLER_PID = {5, 0, 0.3};
    public static double[] ROT_CONTROLLER_PID = {3.2, 0, 0};

    public static double ALIGNMENT_MAX_STOPPED_TRANS_SPEED = 0.1; // m/s robot must be going slower than this in x and y direction to be considered 'stopped' and thus done aligning
    public static double ALIGNMENT_MAX_STOPPED_ROT_SPEED = Units.degreesToRadians(1); // rad/s robot must be going slower than this in x and y direction to be considered 'stopped' and thus done aligning

    public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // rad/s^2
    public static final double WHEEL_RADIUS_MAX_VEL = 0.25; // rad/s
}