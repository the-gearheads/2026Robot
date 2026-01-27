package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.motors.DriveMotor;
import frc.robot.subsystems.swerve.motors.DriveMotorSim;
import frc.robot.subsystems.swerve.motors.SteerMotor;
import frc.robot.subsystems.swerve.motors.SteerMotorSim;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.constants.SwerveConstants.MOTOR_IDS;
import static frc.robot.constants.SwerveConstants.WHEEL_OFFSETS;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleSim implements SwerveModuleIO {

  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController drive;
  private final SimulatedMotorController.GenericMotorController steer;

  String modulePath;

  public SwerveModuleSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;

    this.modulePath = "Swerve/" + moduleSimulation.toString();

    this.drive = moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(60));
    this.steer = moduleSimulation
            .useGenericControllerForSteer()
            .withCurrentLimit(Amps.of(20));
  }

  public void setState(SwerveModuleState state) {
    Logger.recordOutput(modulePath + "/DesiredSwerveStatePreOpt", state);

    state.optimize(Rotation2d.fromRadians(moduleSimulation.getSteerAbsoluteAngle().magnitude()));
    state.cosineScale(Rotation2d.fromRadians(moduleSimulation.getSteerAbsoluteAngle().magnitude()));

    Logger.recordOutput(modulePath + "/DesiredSwerveStatePostOpt", state);

    moduleSimulation.set
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

  public DriveMotor getDrive() {
    return drive;
  }

  public SteerMotor getSteer() {
    return steer;
  }
}