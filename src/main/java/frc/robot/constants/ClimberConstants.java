package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ClimberConstants {

    public static final int CLIMBER_ID = 50;
    public static final int COLOR_SENSOR_ID = 51;
    public static final int CLIMBER_CURRENT_LIMIT = 60;
   // public static final int LASERCAN_ID = 51;

    public static final double CLIMBING_POLE_PROXIMITY_THRESHOLD = 0.53;
    public static final double CLIMB_POLE_MIN_HUE = 0.1;  
    public static final double CLIMB_POLE_MAX_HUE = 0.18;  

    public static final double CLIMB_UP_VOLTAGE = 12.0;
    public static final double CLIMB_DOWN_VOLTAGE = 12.0;

    public static final double MAX_CLIMBER_POS = 106.5;
    public static final double CLIMB_DOWN_POS = 60.48;
    public static final double MIN_CLIMBER_POS = 5.0;

    public static final Pose2d CLIMB_RIGHT_POSE = new Pose2d(0.650, 2.605, Rotation2d.kCW_90deg);
    public static final Pose2d CLIMB_LEFT_POSE = new Pose2d(0.650, 4.770, Rotation2d.kCCW_90deg);
    public static final double CLIMB_SWEEP_SPEED = 0.25;  // m/s
    public static final double CLIMB_IN_SPEED = 0.25;  // m/s

    public static final double MAX_AUTOCLIMB_DIST = 2.16742069; // autoclimb won't happen if we are further away than this. meters
    public static final double AUTOCLIMB_DOWN_PROXIMITY = 0.2; // proximity where the climber will go down pulling up the robot
}