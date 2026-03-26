package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface AimingTarget {
    /* The AimingTarget Manifesto
     *   These are implemented in the targets folder
     *     - instances of those should prolly only be made in ObjectiveTracker.java
     *   Both for logging and to avoid footguns, these classes shouldn't think, all logic should be handled elsewhere
     *   (either objectiveTracker or Shootercalcs) that's why FeederTarget doesn't know if it's left or right
     */
    Translation2d getFieldPosition(); 
    
    Rotation2d getHoodAngle(double distanceMeters);
    double getFlywheelVel(double distanceMeters);
    double getTimeOfFlight(double distanceMeters);
}
