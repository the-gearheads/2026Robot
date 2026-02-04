package frc.robot.subsystems.shooter;

import static frc.robot.constants.ShooterConstants.HOOD_FEEDFORWARD;
import static frc.robot.constants.ShooterConstants.HOOD_RATIO;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodSim extends Hood {
    DCMotor hoodMotor = DCMotor.getNeoVortex(1);
    SingleJointedArmSim hoodSim = new SingleJointedArmSim(
        LinearSystemId.identifyPositionSystem(HOOD_FEEDFORWARD.getKv(), HOOD_FEEDFORWARD.getKa()),
         hoodMotor, HOOD_RATIO,  );

}
