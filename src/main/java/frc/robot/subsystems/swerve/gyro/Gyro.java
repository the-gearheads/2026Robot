package frc.robot.subsystems.swerve.gyro;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface Gyro {
  /**
   * Gets rotation as a Rotation2d
   */
  public Rotation2d getRotation2d();

  /**
   * Gets rotation as a Rotation3d
   */
  public Rotation3d getRotation3d();
  
  /**
   * Gets rotation in degrees
   */
  public double getYaw();
  
  /**
   * Gets angular velocity of yaw, rad/sec
   */
  public double getVelocityYaw();

  public boolean isConnected();
  
  public void reset();
  
  /**
   * Log any other periodic gyro-specific things.
   */
  public default void log() {};
}