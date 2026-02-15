package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
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
    public final String name;
    public final String path;

    public final Transform3d transform;
    
    public final PhotonCamera camera;
    public final PhotonPoseEstimator estimator;

    private final double MAX_PITCHROLL = VisionConstants.MAX_PITCHROLL;
    private final double MAX_Z = VisionConstants.MAX_Z;

    private final double xyStdDevCoefficient = 0.16;
    private final double thetaStdDevCoefficient = 0.2;

    private final AprilTagFieldLayout field;
    Supplier<Pose2d> robotPoseSupplier;
    
    public Camera(AprilTagFieldLayout field, String name, Transform3d transform, Supplier<Pose2d> robotPose){
        this.name = name;
        this.transform = transform;
        this.field = field;
        this.robotPoseSupplier = robotPoseSupplier;
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

    private Optional<Pose3d> filterPose(EstimatedRobotPose estimatedPose) {
        if (DriverStation.isDisabled()) {
            return Optional.of(estimatedPose.estimatedPose);
        }

        Pose3d estPose = estimatedPose.estimatedPose;
        double pitch = estPose.getRotation().getX();
        double roll = estPose.getRotation().getY();

        if (Math.abs(pitch) > MAX_PITCHROLL || Math.abs(roll) > MAX_PITCHROLL
        || Math.abs(estPose.getTranslation().getZ()) > MAX_Z) {
            return Optional.empty();
        }
        
        if (!VisionConstants.FIELD.contains(estPose.toPose2d().getTranslation())) {
            return Optional.empty();
        }

        // optional filtering by total and avg dist
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
    
    public List<VisionObservation> getObservations(SwerveDrivePoseEstimator poseEstimator) {
        boolean posedUnfilitered = false;
        List<VisionObservation> visionObservations = new ArrayList<>();
        List<PhotonPipelineResult> pipelineResults = getPipelineResults();
        Optional<EstimatedRobotPose> poseResult;

        for (PhotonPipelineResult result : pipelineResults) {
            poseResult = estimator.estimateCoprocMultiTagPose(result);
            if (poseResult.isEmpty())
                continue;

            EstimatedRobotPose pose = poseResult.get();
            
            Logger.recordOutput(path + "/EstPoseUnfilitered", pose.estimatedPose);
            posedUnfilitered = true;

            Optional<Pose3d> filteredPose = filterPose(pose);
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
            double xyStdDev = xyStdDevCoefficient * stdDevFactor;
            double thetaStdDev = thetaStdDevCoefficient * stdDevFactor;

            Logger.recordOutput(path + "/XyStdDev", xyStdDev);
            Logger.recordOutput(path + "/ThetaStdDev", thetaStdDev);
            Logger.recordOutput(path + "/NumTargets", numTargets);
            Logger.recordOutput(path + "/AvgTagDist", avgTagDist);
            Logger.recordOutput(path + "/EstPose", pose.estimatedPose);

            var stddevs = MatBuilder.fill(Nat.N3(), Nat.N1(), xyStdDev, xyStdDev, thetaStdDev);
            visionObservations.add(new VisionObservation(pose, stddevs));
        }

        if (visionObservations.isEmpty()) {
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

    public record VisionObservation(
        EstimatedRobotPose poseResult,
        Matrix<N3, N1> stddevs
    ) {}
    
}

