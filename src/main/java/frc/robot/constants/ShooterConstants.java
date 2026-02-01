package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ShooterConstants {
    public static final int MAIN_FLY_ID = 1;
    public static final int FOLLOWER_FLY_ID = 2;
    public static final int HOOD_MOTOR_ID = 3;
    public static final int TOP_ROLLER_ID = 4;
    public static final double[] HOOD_PID = {0, 0, 0};
    public static final Constraints HOOD_CONSTRAINTS = new Constraints(1,1);

}
