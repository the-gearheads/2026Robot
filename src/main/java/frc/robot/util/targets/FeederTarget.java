package frc.robot.util.targets;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.AimingTarget;
import frc.robot.util.AllianceFlipUtil;

public class FeederTarget implements AimingTarget {
    static InterpolatingDoubleTreeMap feedAngleFunction = new InterpolatingDoubleTreeMap();
    static InterpolatingDoubleTreeMap feedVelFunction = new InterpolatingDoubleTreeMap();
    static InterpolatingDoubleTreeMap feedToFFunction = new InterpolatingDoubleTreeMap();

    static {
        for(int i=0; i<ShooterConstants.SHOOT_DISTANCES.length; i++) {  // TODO: make new ones of these
            feedAngleFunction.put(ShooterConstants.SHOOT_DISTANCES[i], ShooterConstants.SHOOT_ANGLES[i]);
        }
        for(int i=0; i<ShooterConstants.SHOOT_SPEEDS.length; i++) {
            feedVelFunction.put(ShooterConstants.SHOOT_DISTANCES[i], ShooterConstants.SHOOT_SPEEDS[i]);
        }
        for(int i=0; i<ShooterConstants.SHOOT_TOFS.length; i++) {
            feedToFFunction.put(ShooterConstants.SHOOT_DISTANCES[i], ShooterConstants.SHOOT_TOFS[i]);
        }
    }

    private final Translation2d location;

    public FeederTarget(Translation2d location) {
        this.location = location;
    }

    @Override
    public Translation2d getFieldPosition() {
        return AllianceFlipUtil.apply(this.location);
    }

    @Override
    public Rotation2d getHoodAngle(double distanceMeters) {
        Rotation2d angle = Rotation2d.fromRadians(feedAngleFunction.get(distanceMeters));
        return angle;
    }

    @Override
    public double getFlywheelVel(double distanceMeters) {
        double velocity = feedVelFunction.get(distanceMeters);
        return velocity;
    }

    @Override
    public double getTimeOfFlight(double distanceMeters) {
        double tof = feedToFFunction.get(distanceMeters);
        return tof;
    }

    @Override
    public String toString() {
        return "FEED " + location.getX() + location.getY();
    }
}
