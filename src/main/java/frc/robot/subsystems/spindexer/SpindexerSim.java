package frc.robot.subsystems.spindexer;

import static frc.robot.constants.SpindexerConstants.FEEDER_GEAR_RATIO;
import static frc.robot.constants.SpindexerConstants.MAINSPINNER_GEAR_RATIO;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class SpindexerSim extends Spindexer{
    DCMotor mainSpinnerGearbox = DCMotor.getNEO(1);
    DCMotor feederGearbox = DCMotor.getNEO(1);

    SparkFlexSim mainSpinnerFlexSim = new SparkFlexSim(mainSpinner, mainSpinnerGearbox);
    SparkFlexSim feederFlexSim = new SparkFlexSim(feeder, feederGearbox);

    FlywheelSim mainSpinnerSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(mainSpinnerGearbox, 0.0043895948, MAINSPINNER_GEAR_RATIO), mainSpinnerGearbox, 0);
    FlywheelSim feederSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(feederGearbox, 0.0043895948, FEEDER_GEAR_RATIO), feederGearbox, 0);

    public SpindexerSim() {
        configure();
    }

    @Override
    public void simulationPeriodic()
    {
        mainSpinnerSim.setInputVoltage(mainSpinnerFlexSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        feederSim.setInputVoltage(feederFlexSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        mainSpinnerSim.update(0.02);
        feederSim.update(0.02);

        mainSpinnerFlexSim.iterate(mainSpinnerSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);
        feederFlexSim.iterate(feederSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);
    }


}
