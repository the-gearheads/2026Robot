package frc.robot.subsystems.shooter;

import static frc.robot.constants.ShooterConstants.HOOD_FEEDFORWARD;
import static frc.robot.constants.ShooterConstants.HOOD_LENGTH_METERS;
import static frc.robot.constants.ShooterConstants.HOOD_MAX_ANGLE;
import static frc.robot.constants.ShooterConstants.HOOD_MIN_ANGLE;
import static frc.robot.constants.ShooterConstants.HOOD_RATIO;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodSim extends Hood {
    DCMotor hoodMotor = DCMotor.getNeoVortex(1);
    SingleJointedArmSim hoodSim = new SingleJointedArmSim(
        LinearSystemId.identifyPositionSystem(HOOD_FEEDFORWARD.getKv(), HOOD_FEEDFORWARD.getKa()),
         hoodMotor, HOOD_RATIO, HOOD_LENGTH_METERS, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE, false, 0,  0, 0 );


    public HoodSim() {}

    @Override
    public void simulationPeriodic() {
        hoodSim.update(.02);
        Logger.recordOutput("Hood/Velocity", getHoodVelocity());
    }


    @Override
    @AutoLogOutput
    public double getHoodVelocity() {
       return hoodSim.getVelocityRadPerSec();
    }

    @Override
    public void setHoodVoltage(double volts) {
        hoodSim.setInputVoltage(volts);
    }




}
