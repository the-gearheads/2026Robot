package frc.robot.subsystems.swerve;

import java.util.Queue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.motors.DriveMotor;
import frc.robot.subsystems.swerve.motors.SteerMotor;

public interface SwerveModuleIO {

    public DriveMotor getDrive();

    public SteerMotor getSteer();

    public void setState(SwerveModuleState state);

    public void runCharacterization(double output);

    public SwerveModulePosition getCurrentModulePosition();

    public SwerveModuleState getCurrentState();

    public void setDriveVolts(double volts);

    public void setSteerVolts(double volts);

    public void voltageOnlyMode(boolean drive, boolean steer);

    public void periodic();

    public void configure();

    public void setBrakeCoast(boolean willBrake);
} 