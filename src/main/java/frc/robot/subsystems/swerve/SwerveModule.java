package frc.robot.subsystems.swerve;

import java.util.Queue;

import org.littletonrobotics.junction.Logger;

import static frc.robot.constants.SwerveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.motors.DriveMotor;
import frc.robot.subsystems.swerve.motors.DriveMotorSim;
import frc.robot.subsystems.swerve.motors.SteerMotor;
import frc.robot.subsystems.swerve.motors.SteerMotorSim;

public class SwerveModule implements SwerveModuleIO {
  public final DriveMotor drive;
  public final SteerMotor steer;

  Rotation2d offset;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  String modulePath;

  public SwerveModule(int id, String moduleName) {
    this.modulePath = "Swerve/" + moduleName;
    this.offset = Rotation2d.fromDegrees(WHEEL_OFFSETS[id]);

    drive = new DriveMotor(MOTOR_IDS[id][0], id, modulePath);
    steer = new SteerMotor(MOTOR_IDS[id][1], id, offset, modulePath);

    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(drive::getPositionOptional);
    turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(steer::getAngleRadiansOptional);
  }

  public void setState(SwerveModuleState state) {
    Logger.recordOutput(modulePath + "/DesiredSwerveStatePreOpt", state);

    state.optimize(steer.getAngle());
    state.cosineScale(steer.getAngle());

    Logger.recordOutput(modulePath + "/DesiredSwerveStatePostOpt", state);

    steer.setAngle(state.angle);
    drive.setSpeed(state.speedMetersPerSecond);
  }

  public void runCharacterization(double output) {
    drive.setVoltage(output);
    steer.setAngle(new Rotation2d());
  }

  public SwerveModulePosition getCurrentModulePosition() {
    return new SwerveModulePosition(drive.getPosition(), steer.getAngle());
  }

  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(drive.getVelocity(), steer.getAngle());
  }

  public void setDriveVolts(double volts) {
    drive.setVoltage(volts);
  }

  public void setSteerVolts(double volts) {
    steer.setVoltage(volts);
  }

  public void voltageOnlyMode(boolean drive, boolean steer) {
    /*
     * disables PID for motors so you can set voltage in peace (for sysid)
     */
    this.drive.setManualVoltageOnly(drive);
    this.steer.setManualVoltageOnly(steer);
  }

  public void periodic() {
    steer.periodic();
    drive.periodic();
    steer.log();
    drive.log();
  }

  public void configure() {
    steer.configure();
    drive.configure();
  }

  public void setBrakeCoast(boolean willBrake) {
    drive.setBrakeCoast(willBrake);
    steer.setBrakeCoast(willBrake);
  }  

  // for 250 hz odometry, currently unused
  private Rotation2d[] getOdometrySteerPositions() {
    var angles = turnPositionQueue
        .stream()
        .map((Double value) -> Rotation2d.fromRadians(value))
        .toArray(Rotation2d[]::new);
    Logger.recordOutput(modulePath + "/odometrySteerPositions", angles);
    turnPositionQueue.clear();
    return angles;
  }

  private double[] getOdometryDrivePositions() {
    var positions = drivePositionQueue
        .stream()
        .mapToDouble((Double value) -> value)
        .toArray();
    Logger.recordOutput(modulePath + "/odometryDrivePositions", positions);
    drivePositionQueue.clear();
    return positions;
  }

  public double[] getOdometryTimestamps() {
    var timestamps = timestampQueue
        .stream()
        .mapToDouble((Double value) -> value)
        .toArray();
    Logger.recordOutput(modulePath + "/odometryTimestamps", timestamps);
    timestampQueue.clear();
    return timestamps;
  }

  public SwerveModulePosition[] getOdometryModPositions() {
    var drivePositions = getOdometryDrivePositions();
    var turnPositions = getOdometrySteerPositions();
    var positions = new SwerveModulePosition[drivePositions.length];
    for (int i = 0; i < drivePositions.length; i++) {
      positions[i] = new SwerveModulePosition(drivePositions[i], turnPositions[i]);
    }
    return positions;
  }

  public DriveMotor getDrive() {
    return drive;
  }

  public SteerMotor getSteer() {
    return steer;
  }
}