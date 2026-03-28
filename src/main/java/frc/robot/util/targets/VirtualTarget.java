package frc.robot.util.targets;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.AimingTarget;

public class VirtualTarget implements AimingTarget {
    public final AimingTarget baseTarget;
    public final Translation2d virtualPosition;

    public VirtualTarget(AimingTarget baseTarget, Translation2d virtualPosition) {
        this.baseTarget = baseTarget;
        this.virtualPosition = virtualPosition;
    }

    @Override
    public Translation2d getFieldPosition() {
        return virtualPosition;
    }

    @Override
    public Rotation2d getHoodAngle(double distance) {
        return baseTarget.getHoodAngle(distance);
    }

    @Override
    public double getFlywheelVel(double distance) {
        return baseTarget.getFlywheelVel(distance);
    }

    @Override
    public double getTimeOfFlight(double distance) {
        return baseTarget.getTimeOfFlight(distance);
    }
}