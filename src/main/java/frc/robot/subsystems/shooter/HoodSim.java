package frc.robot.subsystems.shooter;

import static frc.robot.constants.ShooterConstants.HOOD_FEEDFORWARD;
import static frc.robot.constants.ShooterConstants.HOOD_GEAR_RATIO;
import static frc.robot.constants.ShooterConstants.HOOD_LENGTH_METERS;
import static frc.robot.constants.ShooterConstants.HOOD_MAX_ANGLE;
import static frc.robot.constants.ShooterConstants.HOOD_MIN_ANGLE;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodSim extends Hood {
    DCMotor hoodGearbox = DCMotor.getNeoVortex(1);
    SparkFlexSim flexSim = new SparkFlexSim(hood, hoodGearbox);
    SingleJointedArmSim armSim = new SingleJointedArmSim(
        LinearSystemId.identifyPositionSystem(HOOD_FEEDFORWARD.getKv(), HOOD_FEEDFORWARD.getKa()),
         hoodGearbox, HOOD_GEAR_RATIO, HOOD_LENGTH_METERS, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE, false, 0,  0, 0 );


    public HoodSim() {
        configure();
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInput(flexSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        armSim.update(.02);

        flexSim.iterate(Units.radiansPerSecondToRotationsPerMinute(
            armSim.getVelocityRadPerSec()
        ), RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    }


    @Override
    @AutoLogOutput
    public double getHoodVelocity() {
       return armSim.getVelocityRadPerSec();
    }

    @Override
    public void setHoodVoltage(double volts) {
        armSim.setInputVoltage(volts);
    }
}
