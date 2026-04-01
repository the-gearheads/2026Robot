package frc.robot.subsystems.swerve.gyro;


import org.littletonrobotics.junction.AutoLogOutput;

import com.reduxrobotics.sensors.canandgyro.Vec3Frame;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class GyroSim implements Gyro {

  double yaw;
  double velocityYaw;

  public GyroSim() {
  }

  public void reset() {
    yaw = 0;
  }

  @AutoLogOutput(key="Swerve/simGyro/rotation2d")
  public Rotation2d getRotation2d() {
    return new Rotation2d(yaw);
  }

  @AutoLogOutput(key="Swerve/simGyro/rotation3d")
  public Rotation3d getRotation3d() {
    return new Rotation3d(0, 0, yaw);
  }

  // Sets sim angle, rad.
  public void setYaw(double yaw) {
    this.yaw = yaw;
  }

  @AutoLogOutput(key="Swerve/simGyro/yaw")
  public double getYaw() {
    return yaw;
  }

  @AutoLogOutput(key="Swerve/simGyro/velocityYaw")
  public double getVelocityYaw() {
    return velocityYaw;
  }

  // Sets sim velocity, rad/sec.
  public double setVelocityYaw(double velocity) {
    return velocityYaw = velocity;
  }

  @AutoLogOutput(key="Swerve/simGyro/isConnected")
  public boolean isConnected() {
    return true;
  }

  public Vec3Frame getLinearAcceleration() {
    var zeros = VecBuilder.fill(0.0, 0.0, 0.0);
    return new Vec3Frame(0.0, zeros, 1.0);
  }
}