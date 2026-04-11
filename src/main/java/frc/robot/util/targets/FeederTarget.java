package frc.robot.util.targets;

import static frc.robot.constants.ShooterConstants.FEEDING_ANGLE;
import static frc.robot.constants.ShooterConstants.FEEDING_VEL;
import static frc.robot.constants.ShooterConstants.SHOOT_DISTANCES;

import org.apache.commons.math3.analysis.interpolation.AkimaSplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.AimingTarget;
import frc.robot.util.AllianceFlipUtil;

public class FeederTarget implements AimingTarget {
    private static final AkimaSplineInterpolator interpolator = new AkimaSplineInterpolator();
    private static final PolynomialSplineFunction tofSpline = interpolator.interpolate(ShooterConstants.SHOOT_DISTANCES, ShooterConstants.SHOOT_TOFS);
    private static final PolynomialSplineFunction angleSpline = interpolator.interpolate(ShooterConstants.SHOOT_DISTANCES, ShooterConstants.SHOOT_ANGLES);
    private static final PolynomialSplineFunction velSpline = interpolator.interpolate(ShooterConstants.SHOOT_DISTANCES, ShooterConstants.SHOOT_SPEEDS);
    
    private static final PolynomialSplineFunction tofDerivative = tofSpline.polynomialSplineDerivative();
    
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
        return FEEDING_ANGLE;
    }

    @Override
    public double getFlywheelVel(double distanceMeters) {
        if(distanceMeters <= 6)
        {
        return ShooterConstants.SHORT_FEED_VEL;
        }
        else {
        return FEEDING_VEL;
        }
    }

    @Override
    public double getTimeOfFlight(double distanceMeters) {
        double clampedDist = Math.max(SHOOT_DISTANCES[0], Math.min(distanceMeters, SHOOT_DISTANCES[SHOOT_DISTANCES.length - 1]));
        return tofSpline.value(clampedDist);
    }

    @Override
    public double getTofDerivative(double distanceMeters) {
        double clampedDist = Math.max(SHOOT_DISTANCES[0], Math.min(distanceMeters, SHOOT_DISTANCES[SHOOT_DISTANCES.length - 1]));
        return tofDerivative.value(clampedDist);
    } 

    @Override
    public String toString() {
        return "FEED " + location.getX() + location.getY();
    }
}
