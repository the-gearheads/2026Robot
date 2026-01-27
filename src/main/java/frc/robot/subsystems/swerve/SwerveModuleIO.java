package frc.robot.subsystems.swerve;

import java.util.Queue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.motors.DriveMotor;
import frc.robot.subsystems.swerve.motors.SteerMotor;

public interface SwerveModuleIO {
    
    public final DriveMotor drive;
    public final SteerMotor steer;
    Rotation2d offset;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    String modulePath;

    public void setState(SwerveModuleState state);

    public void runCharacterization(double output);

    public SwerveModulePosition getCurrentModulePosition();

    public SwerveModuleState getCurrentState();

    public void setDriveVolts(double volts);

    public void setSteerVolts(double volts);

    public void voltageOnlyMode(boolean drive, boolean steer);

    public void periodic();

    public void configure();

    private default Rotation2d[] getOdometrySteerPositions() {
        var angles = turnPositionQueue
            .stream()
            .map((Double value) -> Rotation2d.fromRadians(value))
            .toArray(Rotation2d[]::new);
        Logger.recordOutput(modulePath + "/odometrySteerPositions", angles);
        turnPositionQueue.clear();
        return angles;
    }

    private default double[] getOdometryDrivePositions() {
      var positions = drivePositionQueue
          .stream()
          .mapToDouble((Double value) -> value)
          .toArray();
      Logger.recordOutput(modulePath + "/odometryDrivePositions", positions);
      drivePositionQueue.clear();
      return positions;
    }

    public default double[] getOdometryTimestamps() {
      var timestamps = timestampQueue
          .stream()
          .mapToDouble((Double value) -> value)
          .toArray();
      Logger.recordOutput(modulePath + "/odometryTimestamps", timestamps);
      timestampQueue.clear();
      return timestamps;
    }

    public default SwerveModulePosition[] getOdometryModPositions() {
      var drivePositions = getOdometryDrivePositions();
      var turnPositions = getOdometrySteerPositions();
      var positions = new SwerveModulePosition[drivePositions.length];
      for (int i = 0; i < drivePositions.length; i++) {
        positions[i] = new SwerveModulePosition(drivePositions[i], turnPositions[i]);
      }
      return positions;
    }

    public void setBrakeCoast(boolean willBrake);
} 