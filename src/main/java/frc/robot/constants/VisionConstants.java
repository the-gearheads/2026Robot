package frc.robot.constants;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    
    public static final String[] CAMERA_NAMES = {"RA", "ALLAN", "PEARL", "ANNIE"};

    public static final PoseStrategy[] INITIAL_CAMERA_STRATEGIES = {PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR};
    public static final Transform3d[] CAMERA_TRANSFORMS = {
        new Transform3d(),
        new Transform3d(),
        new Transform3d()
    };

    // maybe intrinsics in here sometime

    public static final Rectangle2d FIELD = new Rectangle2d(
        new Translation2d(0, 0),
        new Translation2d(16.58, 8.11)  // vibed from choreo    
    );

    public static final double MAX_PITCHROLL = Units.degreesToRadians(5);
    public static final double MAX_Z = Units.inchesToMeters(7);
}
