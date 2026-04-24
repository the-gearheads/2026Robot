package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Camera.VisionObservation;

public class Vision extends SubsystemBase {
    private Swerve swerve;
    private VisionSim sim = new VisionSim();

    private Camera[] cameras = new Camera[CAMERA_NAMES.length];
    @AutoLogOutput
    private boolean currentlyOnBump = false;

    public Vision(Swerve swerve) {
        this.swerve = swerve;
        PhotonCamera.setVersionCheckEnabled(false); // we using super cool dev version today

        for (int i = 0; i<CAMERA_NAMES.length; i++) {
            cameras[i] = new Camera(FieldConstants.ATFL, CAMERA_NAMES[i], CAMERA_TRANSFORMS[i], swerve::getPose, WRONG_CAMERA_INTRINSICS[i]);
        }

        if (Robot.isSimulation()) {
            for (int i = 0; i<CAMERA_NAMES.length; i++) {
                sim.addCamera(cameras[i]);
            }
        }
    }

    private void addVisionMeasurement(SwerveDrivePoseEstimator poseEstimator, Camera cam, VisionObservation observation) {
        if (VisionConstants.ENABLED) {
            poseEstimator.addVisionMeasurement(observation.poseResult().estimatedPose.toPose2d(), observation.poseResult().timestampSeconds, observation.stddevs());
        }
    } 

    public boolean feedPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
        boolean posed = false;
        for (Camera camera : cameras) {
            for (VisionObservation observation : camera.getObservations(poseEstimator, currentlyOnBump)) {
                addVisionMeasurement(poseEstimator, camera, observation);
                posed = true;
            }
        }

        // $$ \ce{H2O <-> H2O} $$
        return posed;
    }

    @Override
    public void periodic() {
        if (swerve.getGyro3dRotation().getX() > BUMP_ROLL_THRESHOLD.getRadians() || swerve.getGyro3dRotation().getY() > BUMP_PITCH_THRESHOLD.getRadians()) {
            currentlyOnBump = true;
        } else {
            currentlyOnBump = false;
        }

        sim.periodic(swerve.getPoseWheelsOnly());
        for (Camera camera : cameras){
            camera.logCamTransform(swerve.getPose());
        }
    }
    
   
}