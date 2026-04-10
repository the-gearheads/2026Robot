package frc.robot.constants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public final class SpindexerConstants {
    public static final int SPINNER_ID = 35;
    public static final int FEEDER_ID = 36;
    public static final int FLOOBER_ID = 37;

    public static final int SPINNER_CURRENT_LIMIT = 60;
    public static final int FEEDER_CURRENT_LIMIT = 60;
    public static final int FLOOBER_CURRENT_LIMIT = 20;

    public static final double MAINSPINNER_GEAR_RATIO = 9.0/1;
    public static final double FEEDER_GEAR_RATIO = 3.0/1;

    public static final double FEEDER_POS_RATIO = (1.0 / FEEDER_GEAR_RATIO) * (2 * Math.PI);
    public static final double FEEDER_VEL_RATIO = FEEDER_POS_RATIO / 60.0;    

    // public static final double[] FEEDER_PID = {0.008*12.0, 0, 0.001*12.0};
    public static final double[] FEEDER_PID = {0, 0, 0};
    public static final SimpleMotorFeedforward FEEDER_FEEDFORWARD = new SimpleMotorFeedforward(0.15, 0.0172, 0, 0.02);
    public static final double MAX_FEEDER_VEL = FEEDER_FEEDFORWARD.maxAchievableVelocity(12, 0);
    
    public static final double FEED_VELOCITY = Units.rotationsPerMinuteToRadiansPerSecond(2000);
    public static final double FEEDER_SPEED_TOLERANCE = Units.rotationsPerMinuteToRadiansPerSecond(400);
}
