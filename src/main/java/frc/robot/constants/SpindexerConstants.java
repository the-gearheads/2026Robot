package frc.robot.constants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public final class SpindexerConstants {
    public static final int SPINNER_ID = 35;
    public static final int FEEDER_ID = 36;

    public static final int SPINNER_CURRENT_LIMIT = 60;
    public static final int FEEDER_CURRENT_LIMIT = 60;

    public static final double MAINSPINNER_GEAR_RATIO = 15.0/1;
    public static final double FEEDER_GEAR_RATIO = 3.0/1;

    public static final double[] FEEDER_PID = {0, 0, 0};
    public static final SimpleMotorFeedforward FEEDER_FEEDFORWARD = new SimpleMotorFeedforward(0, 0, 0, 0.02); // placeholder values
}

