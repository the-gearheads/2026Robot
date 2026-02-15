package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.*;

import java.util.Optional;


import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Camera.VisionObservation;

public class Vision extends SubsystemBase {
    public static AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private Swerve swerve;

    private Camera[] cameras = new Camera[CAMERA_NAMES.length];

    public Vision(Swerve swerve) {
        this.swerve = swerve;

        for (int i = 0; i<CAMERA_NAMES.length; i++) {
            cameras[i] = new Camera(field, CAMERA_NAMES[i], CAMERA_TRANSFORMS[i], swerve::getPose);
            // sim.addCamera(cameras[i]);
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
        for (Camera camera : cameras){
            camera.logCamTransform(swerve.getPose());
        }
    }
    
   
}