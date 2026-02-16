package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.ArrayList;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;

public class VisionSim {

  VisionSystemSim sim = new VisionSystemSim("main");

  ArrayList<PhotonCameraSim> cameras = new ArrayList<>();

  public VisionSim() {
    if(Robot.isReal()) return;
    AprilTagFieldLayout tagLayout = null;
    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }

    sim.addAprilTags(tagLayout);
  }

  public void addCamera(Camera cam) {
    if(Robot.isReal()) return;
    var camera = new PhotonCameraSim(cam.camera, cam.getSimProperties());
    camera.enableRawStream(false);
    camera.enableProcessedStream(true);
    camera.enableDrawWireframe(false);
    cameras.add(camera);
    sim.addCamera(camera, cam.transform);
  }

  public void periodic(Pose2d robotPose) {
    if(Robot.isReal()) return;
    sim.update(robotPose);
  }
}
