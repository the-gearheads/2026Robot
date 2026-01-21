package frc.robot.subsystems.swerve.gyro;

import static frc.robot.constants.SwerveConstants.BORON_ID;

import org.littletonrobotics.junction.AutoLogOutput;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroFaults;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;

import edu.wpi.first.math.geometry.CoordinateAxis;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class GyroRedux implements Gyro {
  Canandgyro gyro = new Canandgyro(BORON_ID);
  CanandgyroSettings settings = new CanandgyroSettings();
  public GyroRedux() {
    settings.setEphemeral(true);
    gyro.setSettings(settings);
  }

  public void reset() {
    gyro.setYaw(0);
    gyro.setPose(new Rotation3d(), 0);
  }

  @AutoLogOutput(key="Swerve/GyroRedux/rotation2d")
  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  private final CoordinateSystem ENU = new CoordinateSystem(CoordinateAxis.E(), CoordinateAxis.N(), CoordinateAxis.U());

  @AutoLogOutput(key="Swerve/GyroRedux/rotation3d")
  public Rotation3d getRotation3d() {
    return CoordinateSystem.convert(gyro.getRotation3d(), ENU, CoordinateSystem.NWU()); // pitch and yaw are swapped and facing the wrong way
  }

  @AutoLogOutput(key="Swerve/GyroRedux/yaw")
  public double getYaw() {
    return gyro.getMultiturnYaw() * (2 * Math.PI); // rot -> rad
  }

  @AutoLogOutput(key="Swerve/GyroRedux/velocityYaw")
  public double getVelocityYaw() {
    return gyro.getAngularVelocityYaw() * (2 * Math.PI); // rot/sec -> rad/sec
  }

  @AutoLogOutput(key="Swerve/GyroRedux/isConnected")
  public boolean isConnected() {
    return gyro.isConnected();
  }

  @AutoLogOutput(key="Swerve/GyroRedux/activeFaults")
  public CanandgyroFaults getActiveFaults() {
    return gyro.getActiveFaults();
  }

  @AutoLogOutput(key="Swerve/GyroRedux/stickyFaults")
  public CanandgyroFaults getStickyFaults() {
    return gyro.getStickyFaults();
  }

}