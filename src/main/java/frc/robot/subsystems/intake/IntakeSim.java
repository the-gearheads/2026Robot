package frc.robot.subsystems.intake;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeSim extends Intake {
    DCMotor intakeGearbox = DCMotor.getNEO(1);
    DCMotor intakeGearbox = DCMotor.getNEO(1);
    SparkFlexSim flexSim = new SparkFlexSim(hood, hoodGearbox);
    SingleJointedArmSim armSim = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(hoodGearbox, 0.0109192049, HOOD_GEAR_RATIO),
        // LinearSystemId.identifyPositionSystem(0.0069965, 0.022602),
        // LinearSystemId.identifyPositionSystem(HOOD_FEEDFORWARD.getKv(), HOOD_FEEDFORWARD.getKa()),
        hoodGearbox, HOOD_GEAR_RATIO, HOOD_LENGTH_METERS, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE, false, 0);


}
