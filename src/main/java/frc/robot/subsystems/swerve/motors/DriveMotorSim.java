package frc.robot.subsystems.swerve.motors;

import static frc.robot.constants.SwerveConstants.DRIVE_FEEDFORWARD;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.SwerveConstants;

public class DriveMotorSim extends DriveMotor {

  SparkFlexSim simDrive;
  DCMotor flexGearbox = DCMotor.getNeoVortex(1).withReduction(SwerveConstants.DRIVE_RATIO);
  DCMotorSim flywheelSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DRIVE_FEEDFORWARD.getKv(), DRIVE_FEEDFORWARD.getKa()), flexGearbox);

  public DriveMotorSim(int id, int index, String modulePath) {
    super(id, index, modulePath);
    simDrive = new SparkFlexSim(super.flex, flexGearbox);
  }

  @Override
  public void periodic() {
    flywheelSim.setInputVoltage(simDrive.getAppliedOutput() * simDrive.getBusVoltage());
    flywheelSim.update(0.02);
    simDrive.iterate(flywheelSim.getAngularVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);
    super.periodic();
  }

  @Override
  public double getCurrent() {
    return flywheelSim.getCurrentDrawAmps();
  }

}