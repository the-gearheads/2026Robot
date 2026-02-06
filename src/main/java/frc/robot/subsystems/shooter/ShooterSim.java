package frc.robot.subsystems.shooter;

import static frc.robot.constants.ShooterConstants.FLYWHEEL_FEEDFORWARD;
import static frc.robot.constants.ShooterConstants.KICKER_FEEDFORWARD;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ShooterSim extends Shooter {

    // flywheel
    DCMotor flywheelGearbox = DCMotor.getNeoVortex(2);
    SparkFlexSim flywheelFlexSim = new SparkFlexSim(mainFly, flywheelGearbox);  // the flex sim 'watches' the normal flywheel flex, so the integrated pid should work?
    FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(FLYWHEEL_FEEDFORWARD.getKv(), FLYWHEEL_FEEDFORWARD.getKa()),
            flywheelGearbox, 0);

    // top roller
    DCMotor kickerGearbox = DCMotor.getNeoVortex(1);
    SparkFlexSim kickerFlexSim = new SparkFlexSim(kicker, kickerGearbox);
    FlywheelSim kickerSim = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(KICKER_FEEDFORWARD.getKv(), KICKER_FEEDFORWARD.getKa()),
            kickerGearbox, 0);

    public ShooterSim() {
        configure();
    }

    @Override
    public void simulationPeriodic() {
        flywheelSim.setInput(flywheelFlexSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        flywheelSim.update(.02);

        flywheelFlexSim.iterate(flywheelSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
                flywheelSim.getCurrentDrawAmps() + kickerSim.getCurrentDrawAmps()));
    }

    
    @Override
    public void setFlywheelVoltage(double volts) {
        flywheelSim.setInputVoltage(volts);
    }

    @Override
    public void setKickerVoltage(double volts) {
        kickerSim.setInputVoltage(volts);
    }

    @Override
    @AutoLogOutput
    public double getKickerVelocity() {
        return kickerSim.getAngularVelocityRadPerSec();
    }

    @Override
    @AutoLogOutput
    public double getKickerSetpoint() {
        return kickerFlexSim.getSetpoint();
    }

    @Override
    @AutoLogOutput
    public double getFlywheelVelocity() {
        return flywheelSim.getAngularVelocityRadPerSec();
    }

    @Override
    @AutoLogOutput
    public double getFlywheelSetpoint() {
        return flywheelFlexSim.getSetpoint();
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
