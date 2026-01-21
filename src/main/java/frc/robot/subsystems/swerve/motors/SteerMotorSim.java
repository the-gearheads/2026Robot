package frc.robot.subsystems.swerve.motors;

import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Rotation2d;

public class SteerMotorSim extends SteerMotor {
  public SteerMotorSim(int id, int index, Rotation2d offset, String modulePath) {
    super(id, index, offset, modulePath);
  }

  Rotation2d angle = new Rotation2d();

  // override noOffsetGetAngle, periodic, setVoltage

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(targetAngle);
  }

  @Override
  public double getAngleRadians() {
    return targetAngle;
  }

  @Override
  public OptionalDouble getAngleRadiansOptional() {
    return OptionalDouble.of(targetAngle);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void setAngle(Rotation2d angle) {
    super.setAngle(angle);
    this.angle = angle;
  }

  @Override
  public double getCurrent() {
    return 0;
  }
  
}