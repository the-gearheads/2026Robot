package frc.robot.util.targets;

import static frc.robot.constants.ShooterConstants.HUB_ANGLE_ADJUSTMENT;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.AimingTarget;
import frc.robot.util.AllianceFlipUtil;

public class HubTarget implements AimingTarget {
    static InterpolatingDoubleTreeMap shooterAngleFunction = new InterpolatingDoubleTreeMap();
    static InterpolatingDoubleTreeMap shooterVelFunction = new InterpolatingDoubleTreeMap();
    static InterpolatingDoubleTreeMap shooterToFFunction = new InterpolatingDoubleTreeMap();

    static {
        for(int i=0; i<ShooterConstants.SHOOT_DISTANCES.length; i++) {
            shooterAngleFunction.put(ShooterConstants.SHOOT_DISTANCES[i], ShooterConstants.SHOOT_ANGLES[i]);
        }
        for(int i=0; i<ShooterConstants.SHOOT_SPEEDS.length; i++) {
            shooterVelFunction.put(ShooterConstants.SHOOT_DISTANCES[i], ShooterConstants.SHOOT_SPEEDS[i]);
        }
        for(int i=0; i<ShooterConstants.SHOOT_TOFS.length; i++) {
            shooterToFFunction.put(ShooterConstants.SHOOT_DISTANCES[i], ShooterConstants.SHOOT_TOFS[i]);
        }
    }

    @Override
    public Translation2d getFieldPosition(Pose2d robotPose) {
        return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    }

    @Override
    public Rotation2d getHoodAngle(double distanceMeters) {
        Rotation2d angle = Rotation2d.fromRadians(shooterAngleFunction.get(distanceMeters)).plus(HUB_ANGLE_ADJUSTMENT);
        return angle;
    }

    @Override
    public double getFlywheelVel(double distanceMeters) {
        double velocity = shooterVelFunction.get(distanceMeters);
        return velocity;
    }

    @Override
    public double getTimeOfFlight(double distanceMeters) {
        double tof = shooterToFFunction.get(distanceMeters);
        return tof;
    }

}
