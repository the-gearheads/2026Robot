package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;

public final class MiscConstants {
        public static final int PDH_ID = 1;
        public static final boolean isReal = Robot.isReal();
        public static final double JOYSTICK_DEADBAND = 0.005;

        public static final Translation2d LEFT_FEEDING_LOCATION = new Translation2d(2.763,5.752);
        public static final Translation2d RIGHT_FEEDING_LOCATION = new Translation2d(2.763,2.140);

        public static final int SOTM_ITERATIONS = 10;

        public static final double FUEL_PROCESSING_TIME = 3.0;  // seconds
        // this is so we can start shooting tof + real processing time early
        public static final double REAL_FUEL_PROCESSING_TIME = 0.25;  // VERY conservative seconds from ball in hub to ball counted;
        
        public static final double SHIFT_WARNING_TIME = 5.0;  // Controller vibrates X seconds before active

        public static final double G = 9.80665;

        public static final boolean DEATH_MODE = false; //SUPER DUPER IMPORTANT CODE, DO NOT REMOVE OR ROBOT DEATH!!!!
}
