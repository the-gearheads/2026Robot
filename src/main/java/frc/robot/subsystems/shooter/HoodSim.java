package frc.robot.subsystems.shooter;

import static frc.robot.constants.ShooterConstants.HOOD_GEAR_RATIO;
import static frc.robot.constants.ShooterConstants.HOOD_LENGTH_METERS;
import static frc.robot.constants.ShooterConstants.HOOD_MAX_ANGLE;
import static frc.robot.constants.ShooterConstants.HOOD_MIN_ANGLE;


import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodSim extends Hood {
    DCMotor hoodGearbox = DCMotor.getNeoVortex(1);
    SparkFlexSim flexSim = new SparkFlexSim(hood, hoodGearbox);
    SingleJointedArmSim armSim = new SingleJointedArmSim(
        LinearSystemId.identifyPositionSystem(0.0069965, 0.022602),
        // LinearSystemId.identifyPositionSystem(HOOD_FEEDFORWARD.getKv(), HOOD_FEEDFORWARD.getKa()),
        hoodGearbox, HOOD_GEAR_RATIO, HOOD_LENGTH_METERS, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE, false, 0,  0, 0 );


    public HoodSim() {
        configure();
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInputVoltage(flexSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        armSim.update(0.02);

        flexSim.iterate(armSim.getVelocityRadPerSec() / HOOD_GEAR_RATIO, RoboRioSim.getVInVoltage(), 0.02);  // velocity should be in post conversion units, so radians/sec of the Hood

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    }
}
