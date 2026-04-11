package frc.robot.constants;

public class ClimberConstants {

    public static final int CLIMBER_ID = 50;
    public static final int COLOR_SENSOR_ID = 51;
    public static final int CLIMBER_CURRENT_LIMIT = 60;

    public static final double CLIMBING_POLE_PROXIMITY_THRESHOLD = 100.0;  // TODO: Tune this value based on actual sensor readings when near the climbing pole
    public static final double CLIMB_POLE_MIN_HUE = 0.1;  
    public static final double CLIMB_POLE_MAX_HUE = 0.18;  

    public static final double CLIMB_UP_VOLTAGE = 12.0;
    public static final double CLIMB_DOWN_VOLTAGE = 12.0;

    public static final double MAX_CLIMBER_POS = 106.5;
    public static final double CLIMB_DOWN_POS = 60.48;
    public static final double MIN_CLIMBER_POS = 5.0;
}