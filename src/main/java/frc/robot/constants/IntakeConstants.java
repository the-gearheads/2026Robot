package frc.robot.constants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    public static final int DEPLOY_ID = 30;
    public static final int INTAKE_ID = 31;
    public static final int DEPLOY_ENCODER_ID = 34;
    public static final int INTAKE_CURRENT_LIMIT = 60;
    public static final int DEPLOY_CURRENT_LIMIT = 60;

    public static final double[] DEPLOY_PID = {0.5, 0, 0};
    public static final Constraints DEPLOY_CONSTRAINTS = new Constraints(
        Units.degreesToRadians(1500), // per second; max vel
        Units.degreesToRadians(2500)  //  per sec^2; max accel
    );


    public static final double[] INTAKE_PID = {0, 0, 0};
    public static final SimpleMotorFeedforward INTAKE_FEEDFORWARD = new SimpleMotorFeedforward(0, 0, 0);

    public static final float DEPLOY_ABS_ENC_POS_FACTOR = (float)(2 * Math.PI);
    public static final float DEPLOY_ABS_ENC_VEL_FACTOR = (float)(DEPLOY_ABS_ENC_POS_FACTOR / 60.0);  // why tf does it want a float TODO: complain to rev
    public static final float DEPLOY_ABS_ENC_OFFSET = (float)0.305;  // this must be in rotations

    public static final double INTAKE_GEAR_RATIO = 1;

    public static final double DEPLOY_GEAR_RATIO = (74.0 + (2.0/3.0)) /1.0;  // placeholder
    public static final double DEPLOY_POS_FACTOR = (1/DEPLOY_GEAR_RATIO)*(2 * Math.PI);
    public static final double DEPLOY_VEL_FACTOR = DEPLOY_POS_FACTOR / 60.0;

    public static final double DEPLOY_LENGTH = Units.inchesToMeters(21);  // not exact

    public static final Rotation2d DEPLOY_MIN_ANGLE = Rotation2d.fromDegrees(-1.5);  // it sinks into the bumper a bit to hold position
    public static final Rotation2d DEPLOY_MAX_ANGLE = Rotation2d.fromDegrees(84);

    public static final Rotation2d DEPLOY_MIN_SYSID_ANGLE = Rotation2d.fromDegrees(4);
    public static final Rotation2d DEPLOY_MAX_SYSID_ANGLE = Rotation2d.fromDegrees(54);

    public static final Rotation2d DEPLOY_HOLD_ANGLE = Rotation2d.fromDegrees(0);
    public static final Rotation2d DEPLOY_SHIMMY_ANGLE = Rotation2d.fromDegrees(60);
    public static final Rotation2d DEPLOY_ANGLE_TOLERANCE = Rotation2d.fromDegrees(5);
    public static final Rotation2d DEPLOY_SHIMMY_TOLERANCE = Rotation2d.fromDegrees(8);

    public static final double INTAKE_VELOCITY = 5250;


}