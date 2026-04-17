package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ClimberConstants {

    public static final int CLIMBER_ID = 50;
    public static final int CLIMBER_CURRENT_LIMIT = 60;

    public static final double CLIMB_UP_VOLTAGE = 12.0;
    public static final double CLIMB_DOWN_VOLTAGE = 12.0;

    public static final double MAX_CLIMBER_POS = 106.5;
    public static final double CLIMB_DOWN_POS = 60.48;
    public static final double MIN_CLIMBER_POS = 5.0;

    public static final Pose2d CLIMB_RIGHT_POSE = new Pose2d(0.95, 2.788, Rotation2d.kCW_90deg);
    public static final Pose2d CLIMB_LEFT_POSE = new Pose2d(1.25, 4.731, Rotation2d.kCCW_90deg);
    public static final double CLIMB_SWEEP_SPEED = 0.3;  // m/s
    public static final double CLIMB_IN_SPEED = 0.4;  // m/s

    public static final double MAX_AUTOCLIMB_DIST = 2.16742069; // autoclimb won't happen if we are further away than this. meters
}