package frc.robot.subsystems.shooter;



import static frc.robot.constants.ShooterConstants.FLYWHEEL_GEAR_RATIO;
import static frc.robot.constants.ShooterConstants.KICKER_GEAR_RATIO;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ShooterSim extends Shooter {

    
    // flywheel
    // DCMotor + flex is fundamentally broken in some way, and when FlywheelFlexSIm calls flywheelGearbox.getCurrent(speedRadiansperSec) it does it using the wrong units and without reduction
    // so the current draw appears completely incorrect, we'll just use flywheelSim current draw instead
    DCMotor flywheelGearbox = DCMotor.getNeoVortex(2);
    SparkFlexSim flywheelFlexSim = new SparkFlexSim(mainFly, flywheelGearbox);
    FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(flywheelGearbox, 0.002, FLYWHEEL_GEAR_RATIO),
            // LinearSystemId.identifyVelocitySystem(0.04553, 0.0001929),
            // LinearSystemId.identifyVelocitySystem(FLYWHEEL_FEEDFORWARD.getKv(), FLYWHEEL_FEEDFORWARD.getKa()),
            flywheelGearbox, 0);

    // top roller
    DCMotor kickerGearbox = DCMotor.getNeoVortex(1);
    SparkFlexSim kickerFlexSim = new SparkFlexSim(kicker, kickerGearbox);
    FlywheelSim kickerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(kickerGearbox, 0.002, KICKER_GEAR_RATIO),
            // LinearSystemId.identifyVelocitySystem(KICKER_FEEDFORWARD.getKv(), KICKER_FEEDFORWARD.getKa()),
            // LinearSystemId.identifyVelocitySystem(0.04553, 0.0001929),
            kickerGearbox, 0);

    public ShooterSim() {
        configure();
    }

    @Override
    public void simulationPeriodic() {
        flywheelSim.setInputVoltage(flywheelFlexSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        kickerSim.setInputVoltage(kickerFlexSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        flywheelSim.update(0.02);
        kickerSim.update(0.02);

        kickerFlexSim.iterate(Units.rotationsPerMinuteToRadiansPerSecond(kickerSim.getAngularVelocityRPM()), RoboRioSim.getVInVoltage(), 0.02);
        flywheelFlexSim.iterate(Units.rotationsPerMinuteToRadiansPerSecond(flywheelSim.getAngularVelocityRPM()), RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
                flywheelSim.getCurrentDrawAmps() + kickerSim.getCurrentDrawAmps()));
        // RoboRioSim.setVInVoltage(12);
    }

    @Override
    @AutoLogOutput
    public double getFlywheelCurrent() {
        return flywheelSim.getCurrentDrawAmps();
    }

    @Override
    @AutoLogOutput
    public double getKickerCurrent() {
        return kickerSim.getCurrentDrawAmps();
    }
}
