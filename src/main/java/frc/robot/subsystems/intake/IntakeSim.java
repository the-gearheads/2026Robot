package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.INTAKE_GEAR_RATIO;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class IntakeSim extends Intake {
    DCMotor intakeGearbox = DCMotor.getNEO(1);
    DCMotor deployGearbox = DCMotor.getNeoVortex(1);
    SparkMaxSim intakeFlexSim = new SparkMaxSim(intake, intakeGearbox);    

    FlywheelSim intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(intakeGearbox, 0.0043895948, INTAKE_GEAR_RATIO), intakeGearbox, 0);

    public IntakeSim() {
        configure();
    }

    @Override
    public void simulationPeriodic() {        
        intakeSim.setInputVoltage(intakeFlexSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        intakeSim.update(0.02);

        intakeFlexSim.iterate(intakeSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);  // really this shjo
    }
}
