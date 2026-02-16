package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.*;




import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Camera.VisionObservation;

public class Vision extends SubsystemBase {
    public static AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private Swerve swerve;
    private VisionSim sim = new VisionSim();

    private Camera[] cameras = new Camera[CAMERA_NAMES.length];

    public Vision(Swerve swerve) {
        this.swerve = swerve;

        for (int i = 0; i<CAMERA_NAMES.length; i++) {
            cameras[i] = new Camera(field, CAMERA_NAMES[i], CAMERA_TRANSFORMS[i], swerve::getPose, CAMERA_INTRINSICS[i]);
        }

        if (Robot.isSimulation()) {
            for (int i = 0; i<CAMERA_NAMES.length; i++) {
                sim.addCamera(cameras[i]);
            }
        }
    }

    private void addVisionMeasurement(SwerveDrivePoseEstimator poseEstimator, Camera cam, VisionObservation observation) {
        poseEstimator.addVisionMeasurement(observation.poseResult().estimatedPose.toPose2d(), observation.poseResult().timestampSeconds, observation.stddevs());
    } 

    public boolean feedPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
        boolean posed = false;
        for (Camera camera : cameras) {
            for (VisionObservation observation : camera.getObservations(poseEstimator)) {
                addVisionMeasurement(poseEstimator, camera, observation);
                posed = true;
            }
        }

        // $$ \ce{H2O <-> H2O} $$
        return posed;
    }

    @Override
    public void periodic() {
        sim.periodic(swerve.getPoseWheelsOnly());
        for (Camera camera : cameras){
            camera.logCamTransform(swerve.getPose());
        }
    }
    
   
}