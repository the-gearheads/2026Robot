package frc.robot.constants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public final class SpindexerConstants {
    public static final int SPINNER_ID = 35;
    public static final int FEEDER_ID = 36;

    public static final int SPINNER_CURRENT_LIMIT = 60;
    public static final int FEEDER_CURRENT_LIMIT = 60;

    public static final double MAINSPINNER_GEAR_RATIO = 15.0/1;
    public static final double FEEDER_GEAR_RATIO = 3.0/1;

    public static final double[] FEEDER_PID = {0.008*12.0, 0, 0.001*12.0};
    public static final SimpleMotorFeedforward FEEDER_FEEDFORWARD = new SimpleMotorFeedforward(0.15, 0.0172, 0, 0.02);
    public static final double MAX_FEEDER_VEL = FEEDER_FEEDFORWARD.maxAchievableVelocity(12, 0);
    
    public static final double FEED_VELOCITY = MAX_FEEDER_VEL * 0.8;
    public static final double FEEDER_SPEED_TOLERANCE = Units.rotationsPerMinuteToRadiansPerSecond(500);
}
