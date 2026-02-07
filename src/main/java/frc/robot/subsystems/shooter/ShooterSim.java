package frc.robot.subsystems.shooter;


import static frc.robot.constants.ShooterConstants.FLYWHEEL_GEAR_RATIO;
import static frc.robot.constants.ShooterConstants.FLYWHEEL_VEL_FACTOR;
import static frc.robot.constants.ShooterConstants.KICKER_GEAR_RATIO;
import static frc.robot.constants.ShooterConstants.KICKER_VEL_FACTOR;

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
            LinearSystemId.createFlywheelSystem(flywheelGearbox, 0.002, 1/FLYWHEEL_GEAR_RATIO),
            // LinearSystemId.identifyVelocitySystem(0.04553, 0.0001929),
            // LinearSystemId.identifyVelocitySystem(FLYWHEEL_FEEDFORWARD.getKv(), FLYWHEEL_FEEDFORWARD.getKa()),
            flywheelGearbox, 0);

    // top roller
    DCMotor kickerGearbox = DCMotor.getNeoVortex(1);
    SparkFlexSim kickerFlexSim = new SparkFlexSim(kicker, kickerGearbox);
    FlywheelSim kickerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(kickerGearbox, 0.002, 1/KICKER_GEAR_RATIO),
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

        kickerFlexSim.iterate(kickerSim.getAngularVelocityRPM() * KICKER_VEL_FACTOR, RoboRioSim.getVInVoltage(), 0.02);
        flywheelFlexSim.iterate(flywheelSim.getAngularVelocityRPM() * FLYWHEEL_VEL_FACTOR, RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
                flywheelSim.getCurrentDrawAmps() + kickerSim.getCurrentDrawAmps()));
    }

    
    // @Override
    // public void setFlywheelVoltage(double volts) {
    // }

    // @Override
    // public void setKickerVoltage(double volts) {
    //     kickerSim.setInputVoltage(volts);
    // }

    // @Override
    // @AutoLogOutput
    // public double getKickerVelocity() {
    //     return kickerSim.getAngularVelocityRadPerSec();
    // }

    // @Override
    // @AutoLogOutput
    // public double getKickerSetpoint() {
    //     return kickerFlexSim.getSetpoint();
    // }

    // @Override
    // @AutoLogOutput
    // public double getFlywheelVelocity() {
    //     return flywheelSim.getAngularVelocityRadPerSec();
    // }

    // @Override
    // @AutoLogOutput
    // public double getFlywheelSetpoint() {
    //     return flywheelFlexSim.getSetpoint();
    // }

    // @Override
    // @AutoLogOutput
    // public double getFlywheelCurrent() {
    //     return flywheelSim.getCurrentDrawAmps();
    // }

    // @Override
    // @AutoLogOutput
    // public double getKickerCurrent() {
    //     return kickerSim.getCurrentDrawAmps();
    // }
}
