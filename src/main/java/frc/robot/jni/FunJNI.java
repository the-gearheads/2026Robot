package frc.robot.jni;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Demo class for loading the driver via JNI.
 */
public class FunJNI {
    static boolean libraryLoaded = false;

    /**
     * Helper class for determining whether or not to load the driver on static
     * initialization.
     */
    public static class Helper {
        private static AtomicBoolean extractOnStaticLoad = new AtomicBoolean(true);

        /**
         * Get whether to load the driver on static init.
         * 
         * @return true if the driver will load on static init
         */
        public static boolean getExtractOnStaticLoad() {
            return extractOnStaticLoad.get();
        }

        /**
         * Set whether to load the driver on static init.
         * 
         * @param load the new value
         */
        public static void setExtractOnStaticLoad(boolean load) {
            extractOnStaticLoad.set(load);
        }
    }

    static {
        if (Helper.getExtractOnStaticLoad()) {
            System.loadLibrary("FunDriver");
            libraryLoaded = true;
        }
    }

    /**
     * Force load the library.
     */
    public static synchronized void forceLoad() {
        if (libraryLoaded) {
            return;
        }
        System.loadLibrary("FunDriver");
        libraryLoaded = true;
    }

    /**
     * Tells the driver to initialize.
     * This is a demo of a native JNI method from the driver.
     * 
     * @return the int returned by the driver
     * @see "VendorJNI.cpp"
     */
    public static native int initialize();


    /**
     * Solves swerve!!
     * okay actually it just gives you [lerp_xy, lerp_rot] if you give it current, desired velocities.
     * Uses the style of 254setpointgenerator so can't take away speed you already have somehow.
     * @param currentVel [vx, vy, omega]
     * @param desiredVel [vx, vy, omega]
     * @return [lerp_xy, lerp_rot]
     */
    public static native double[] solveSwerve(double[] currentVel, double[] desiredVel);
}