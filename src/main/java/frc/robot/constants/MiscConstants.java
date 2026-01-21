package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class MiscConstants {
     public static final double JOYSTICK_DEADBAND = 0.005;
  public static final int BREAK_COAST_BUTTON_PORT = 9;

  public static final double DISTANCE_TO_REEF = Units.inchesToMeters(15.75);
  public static final double AUTO_ALIGN_FILTER_ANGLE = Units.degreesToRadians(70);
  public static final double REEF_FACE_LENGTH = Units.inchesToMeters(36.792600);
  public static final double MAX_REEF_LINEUP_DIST = 0.7;
  public static final double VECTOR_ERROR_SCALAR = 10;
  public static final double MAX_AUTO_VELOCITY = 2; // m/s;

  public static final double AUTO_ALIGN_ANGLE_THRESHOLD = Units.degreesToRadians(25);
  public static final double AUTO_ALIGN_DIST_THRESHOLD = 1;

  public static final Rotation2d SWERVE_ALIGN_ROT_TOLERANCE = Rotation2d.fromDegrees(2);
  public static final double SWERVE_ALIGN_DIST_TOLERANCE = Units.inchesToMeters(0.75);

  public static boolean AUTO_ALIGN_ENABLED = true;
  public static double BARGE_ALIGN_DEBOUNCE_TIME = 0.13;

  public static final double BARGE_ALIGN_X = 7.654112;

  // robot constants, unused; for reference
  public static final double ROBOT_MOI = 3.070789055; // kg * m^2   - for rinzlers drive base
  public static final double ROBOT_MASS = 31.5; // kg
}
