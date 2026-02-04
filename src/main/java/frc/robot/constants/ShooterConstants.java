package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ShooterConstants {
    public static final int MAIN_FLY_ID = 1;
    public static final int FOLLOWER_FLY_ID = 2;
    public static final int HOOD_MOTOR_ID = 3;
    public static final int TOP_ROLLER_ID = 4;

    public static final double HOOD_RATIO = 1;
    public static final double FLYWHEEL_RATIO = 1;
    public static final double TOP_FLY_RATIO = 1;

    public static final double[] FLYWHEEL_PID = {0, 0, 0};
    public static final SimpleMotorFeedforward FLYWHEEL_FEEDFORWARD = new SimpleMotorFeedforward(0, 0, 0);

    public static final double[] TOPFLY_PID = {0, 0, 0};
    public static final SimpleMotorFeedforward TOPFLY_FEEDFORWARD = new SimpleMotorFeedforward(0, 0, 0);

    public static final double[] HOOD_PID = {0, 0, 0};
    public static final ArmFeedforward HOOD_FEEDFORWARD = new ArmFeedforward(0, 0, 0, 0);

}
