package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.DEPLOY_LENGTH;
import static frc.robot.constants.IntakeConstants.DEPLOY_MAX_ANGLE;
import static frc.robot.constants.IntakeConstants.DEPLOY_MIN_ANGLE;
import static frc.robot.constants.IntakeConstants.DEPLOY_GEAR_RATIO;
import static frc.robot.constants.IntakeConstants.INTAKE_GEAR_RATIO;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeSim extends Intake {
    DCMotor intakeGearbox = DCMotor.getNEO(1);
    DCMotor deployGearbox = DCMotor.getNEO(1);
    SparkMaxSim intakeFlexSim = new SparkMaxSim(intake, intakeGearbox);
    SparkFlexSim deployFlexSim = new SparkFlexSim(deploy, deployGearbox);
    
    SingleJointedArmSim deploySim = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(deployGearbox, 0.346396007, 60.0), 
        // LinearSystemId.identifyPositionSystem(0.0069965, 0.022602),
        // LinearSystemId.identifyPositionSystem(HOOD_FEEDFORWARD.getKv(), HOOD_FEEDFORWARD.getKa()),
        deployGearbox, 60.0, DEPLOY_LENGTH, DEPLOY_MIN_ANGLE.getRadians(), DEPLOY_MAX_ANGLE.getRadians(), false, 0);

    FlywheelSim intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(intakeGearbox, 0.0043895948, INTAKE_GEAR_RATIO), deployGearbox, 0);

    public IntakeSim() {
        configure();
    }

    @Override
    public void simulationPeriodic() {
        intakeSim.setInputVoltage(intakeFlexSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        intakeSim.update(0.02);

        deploySim.setInputVoltage(deployFlexSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        deploySim.update(0.02);

        deployFlexSim.iterate(deploySim.getVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);  // velocity should be in post conversion units, so radians/sec of the Hood
        intakeFlexSim.iterate(intakeSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);  // really this shjo

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(deploySim.getCurrentDrawAmps() + intakeSim.getCurrentDrawAmps()));
    }
}
