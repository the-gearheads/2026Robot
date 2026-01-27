package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleSim implements SwerveModuleIO {
    
    private final SwerveModuleSimulation moduleSimulation;
    // reference to the simulated drive motor
    private final SimulatedMotorController.GenericMotorController driveMotor;
    // reference to the simulated turn motor
    private final SimulatedMotorController.GenericMotorController turnMotor;    


}