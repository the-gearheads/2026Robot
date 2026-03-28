package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import static frc.robot.constants.VisionConstants.THETA_STDDEV_COEF;
import static frc.robot.constants.VisionConstants.XY_STDDEV_COEF;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.VisionConstants;

public class Camera {
    final String name;
    final String path;

    final Transform3d transform;
    final CameraIntrinsics simIntrinsics;

    final PhotonCamera camera;
    final PhotonPoseEstimator estimator;

    final double MAX_PITCHROLL = VisionConstants.MAX_PITCHROLL;
    final double MAX_Z = VisionConstants.MAX_Z;        
    final AprilTagFieldLayout field;
        
    public Camera(AprilTagFieldLayout field, String name, Transform3d transform, Supplier<Pose2d> robotPose, CameraIntrinsics intrinsics){
        this.name = name;
        this.transform = transform;
        this.field = field;
        this.simIntrinsics = intrinsics;
        path = "Vision/" + name.replace("_", "");

        camera = new PhotonCamera(name);
        estimator = new PhotonPoseEstimator(field, transform);
    }

    public List<PhotonPipelineResult> getPipelineResults() {
        return camera.getAllUnreadResults();
    }

    public void logCamTransform(Pose2d robotPose) {
        Pose3d camPose = new Pose3d(robotPose);
        camPose = camPose.transformBy(transform);
        Logger.recordOutput(path + "/CamTranform", camPose);
    }

    private Optional<Pose3d> filterPose(EstimatedRobotPose estimatedPose, boolean onBump) {
        Pose3d estPose = estimatedPose.estimatedPose;
        double pitch = estPose.getRotation().getX();
        double roll = estPose.getRotation().getY();
        if (!VisionConstants.FIELD.contains(estPose.toPose2d().getTranslation())) {
            return Optional.empty();
        }

        if (DriverStation.isDisabled()) {
            return Optional.of(estimatedPose.estimatedPose);  // only do yaw and roll checks if we are enabled so that we can make sure vision inits
        }


        
        if (Math.abs(pitch) > MAX_PITCHROLL || Math.abs(roll) > MAX_PITCHROLL
        || Math.abs(estPose.getTranslation().getZ()) > MAX_Z && !onBump) {
            return Optional.empty();
        }

        // optional filtering by ambiguity and avg dist
        // double totalDist = 0;
        // for (var target : estimatedPose.targetsUsed) {
        //     totalDist += target.bestCameraToTarget.getTranslation().getNorm();
        //     if (target.getPoseAmbiguity() > MAX_TAG_AMBIGUITY) {
        //         return Optional.empty();
        //     }
        // }
        // double averageDist = totalDist / estimatedPose.targetsUsed.size();
        // if (averageDist > MAX_AVG_DIST && Robot.isReal()) { // Sim cameras can see almost infinitely far
        //     return Optional.empty();
        // }

        return Optional.of(estPose);
    }
    
    public List<VisionObservation> getObservations(SwerveDrivePoseEstimator poseEstimator, boolean onBump) {
        boolean posedUnfilitered = false;
        List<VisionObservation> visionObservations = new ArrayList<>();
        List<PhotonPipelineResult> pipelineResults = getPipelineResults();
        Optional<EstimatedRobotPose> poseResult;
        

        for (PhotonPipelineResult result : pipelineResults) {
            
            poseResult = estimator.estimateCoprocMultiTagPose(result);
            if (poseResult.isEmpty()) {
                if (onBump) {
                    poseResult = estimator.estimateLowestAmbiguityPose(result);
                } else {
                    poseResult = estimator.estimateClosestToCameraHeightPose(result);
                }
                if (poseResult.isEmpty()) continue;

                // for now we're not gonnause cconstrained PNP, we can add it if necessary, but I think we'll be getting multiple tags 90% of the time anyway
                // if (!onBump && USE_CONSTRAINED_PNP) {
                //     poseResult = estimator.estimateConstrainedSolvepnpPose(result, camera.getCameraMatrix(), camera.getDistCoeffs(), poseResult);
                // }
            }
            
            EstimatedRobotPose pose = poseResult.get();
            
            Logger.recordOutput(path + "/EstPoseUnfilitered", pose.estimatedPose);
            posedUnfilitered = true;

            Optional<Pose3d> filteredPose = filterPose(pose, onBump);
            if (filteredPose.isEmpty())
                continue;
            
            int[] targetsUsed = new int[pose.targetsUsed.size()];
            for(int i = 0; i < pose.targetsUsed.size(); i++) {
                targetsUsed[i] = pose.targetsUsed.get(i).getFiducialId();
            }
            Logger.recordOutput(path + "/TargetsUsed", targetsUsed);

            int numTargets = pose.targetsUsed.size();
            double avgTagDist = 0;
            for (var target : result.targets) {
                avgTagDist += target.bestCameraToTarget.getTranslation().getNorm();
            }
            avgTagDist /= numTargets;

            double stdDevFactor = Math.pow(avgTagDist, 2.0) / numTargets;
            double xyStdDev = XY_STDDEV_COEF * stdDevFactor;
            double thetaStdDev = THETA_STDDEV_COEF * stdDevFactor;

            if (onBump) {
                thetaStdDev = Double.POSITIVE_INFINITY;
                // could also increase xystddev by a factor of 2 or something; but honestly considering our odometry will be COMPLETELY wrong when midair, may as well just rely on vision
            }

            Logger.recordOutput(path + "/XyStdDev", xyStdDev);
            Logger.recordOutput(path + "/ThetaStdDev", thetaStdDev);
            Logger.recordOutput(path + "/NumTargets", numTargets);
            Logger.recordOutput(path + "/AvgTagDist", avgTagDist);
            Logger.recordOutput(path + "/EstPose", pose.estimatedPose);

            var stddevs = MatBuilder.fill(Nat.N3(), Nat.N1(), xyStdDev, xyStdDev, thetaStdDev);
            visionObservations.add(new VisionObservation(pose, stddevs));
        }

        if (visionObservations.isEmpty()) {
            // technically our logging here is broken; because in the list of camera obervations were logging the same things for each one
            // meaning only the last one will actually get logged
            // but its prob fine
            Logger.recordOutput(path + "/XyStdDev", -1d);
            Logger.recordOutput(path + "/ThetaStdDev", -1d);
            Logger.recordOutput(path + "/NumTargets", 0);
            Logger.recordOutput(path + "/AvgTagDist", -1d);
            Logger.recordOutput(path + "/EstPose", new Pose3d(new Translation3d(-100, -100, -100), new Rotation3d()));
            if(!posedUnfilitered) {
                Logger.recordOutput(path + "/EstPoseUnfiltered", new Pose3d(new Translation3d(-100, -100, -100), new Rotation3d()));
            }
            Logger.recordOutput(path + "/TagPoses", new Pose3d[0]);
            Logger.recordOutput(path + "/TargetsUsed", new int[0]);
            return List.of();
        }

        return visionObservations;
    }

    public SimCameraProperties getSimProperties() {
        SimCameraProperties properties = new SimCameraProperties();
        properties.setCalibration(simIntrinsics.resX, simIntrinsics.resY, simIntrinsics.getCameraMatrix(),
            simIntrinsics.getDistCoeffs());

        // Approximate detection noise with average and standard deviation error in
        // pixels.
        properties.setCalibError(1, 0.5);
        // Set the camera image capture framerate (Note: this is limited by robot loop
        // rate).
        properties.setFPS(50);
        // The average and standard deviation in milliseconds of image data latency.
        properties.setAvgLatencyMs(35);
        properties.setLatencyStdDevMs(7);

        return properties;
    }

    public record VisionObservation(
        EstimatedRobotPose poseResult,
        Matrix<N3, N1> stddevs
    ) {}
    
}

