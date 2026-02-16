package frc.robot.constants;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.CameraIntrinsics;

public class VisionConstants {
    
    public static final String[] CAMERA_NAMES = {"RA", "ALLAN", "PEARL", "ANNIE"};

    public static final PoseStrategy[] INITIAL_CAMERA_STRATEGIES = {PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR};
    public static final Transform3d[] CAMERA_TRANSFORMS = {
        new Transform3d(),
        new Transform3d(),
        new Transform3d(),
        new Transform3d()
    };

    // maybe intrinsics in here sometime
    public static final CameraIntrinsics[] CAMERA_INTRINSICS = {
        new CameraIntrinsics(  // RA
            1280, 800,
            906.1235271017885, 906.3131911811712, 647.0631548443764, 401.6428917491316,
            new double[] {0.04506187328353074,-0.051275911330163695,-4.2370271745676663E-5,-0.0010494597718615677,-0.012449373353808963,-0.001374223570752213,0.00394799518717367,0.0012915739517360477}),
        new CameraIntrinsics(  // ALLAN
            1280, 800,
            902.2608474853896, 902.2161068480787,
            609.9129007473787, 392.80120315523453,
            new double[] {0.0525716608175405,-0.0869035197754647,5.185957545931161E-4,-8.252421873108142E-4,0.02868722144643985,-0.0021021187652514443,0.0038484340525384636,-7.308988653147572E-4}),
        new CameraIntrinsics(  // PEARL TODO: update pearl camera intrinsics
            1600, 1200,
            1286.9714942340822, 1286.958397164024,
            801.3013669030433, 643.6657753564633,
            new double[] {0.03330603946791099,-0.028149602298339575,-3.2480645802923945E-4,-9.9467580835956E-5,-0.01790312270432083,-5.6026682560497186E-5,0.004799695107552726,0.002517423938318154}),
        new CameraIntrinsics(
            1280, 800,
            899.4553655192133, 899.4735752263247,
            666.7186432214919, 417.9391799962286,
            new double[] {0.062065388531203025,-0.10281671805343962,3.438441843174614E-4,2.0050773049833584E-4,0.034730722045643904,-0.0020194597310821023,0.0037208221845576026,-0.0011868328692886513})
    };

    public static final Rectangle2d FIELD = new Rectangle2d(
        new Translation2d(0, 0),
        new Translation2d(16.58, 8.11)  // vibed from choreo    
    );

    public static final double MAX_PITCHROLL = Units.degreesToRadians(5);
    public static final double MAX_Z = Units.inchesToMeters(7);
}
