package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.SplineEncoderSim;

public class DeploySim extends Deploy {
  DCMotor deployGearbox = DCMotor.getNEO(1);
  SparkFlexSim deployFlexSim = new SparkFlexSim(deploy, deployGearbox);

  SingleJointedArmSim deploySim = new SingleJointedArmSim(
      LinearSystemId.createSingleJointedArmSystem(deployGearbox, 0.346396007, 60.0),
      // LinearSystemId.identifyPositionSystem(0.0069965, 0.022602),
      // LinearSystemId.identifyPositionSystem(HOOD_FEEDFORWARD.getKv(),
      // HOOD_FEEDFORWARD.getKa()),
      deployGearbox, 60.0, DEPLOY_LENGTH, DEPLOY_MIN_ANGLE.getRadians(), DEPLOY_MAX_ANGLE.getRadians(), false, 0);

    SplineEncoderSim deployEncoderSim = new SplineEncoderSim(deploySplineEncoder);

  @Override
  public void simulationPeriodic() {
    deploySim.setInputVoltage(deployFlexSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    deploySim.update(0.02);
    
    // velocity should be in post conversion units,so radians/sec of the Hood
    deployFlexSim.iterate(deploySim.getVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);
    deployEncoderSim.setPosition(deployFlexSim.getPosition());
    deployEncoderSim.setAngle(deployFlexSim.getPosition());
    deployEncoderSim.setVelocity(deployFlexSim.getVelocity());

  }

}
