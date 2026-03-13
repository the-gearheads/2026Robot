package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;

public final class MiscConstants {
        public static final int PDH_ID = 1;
        public static final boolean isReal = Robot.isReal();
        public static final double JOYSTICK_DEADBAND = 0.005;

        public static final Translation2d LEFT_FEEDING_LOCATION = new Translation2d(2.763,5.752);
        public static final Translation2d RIGHT_FEEDING_LOCATION = new Translation2d(2.763,2.140);
        public static final Translation2d OVER_FEEDING_LOCATION = new Translation2d(2.552, FieldConstants.fieldWidth/2.0);
}
