package frc.robot.commands;

import frc.robot.constants.SwerveConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.swerve.Swerve;

public class Teleop extends Command {
    Swerve swerve;

    public Teleop(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize()  {
        //runs once when we make the command
    }

    @Override
    public void execute() {
        //runs every 20 milliseconds as long as teleop is running
    double x = Controllers.driverController.getTranslateXAxis();
    double y = Controllers.driverController.getTranslateYAxis();
    double rot = Controllers.driverController.getRotateAxis();

    double xSpeed = Math.signum(x) * Math.pow(x, 2);
    double ySpeed = Math.signum(y) * Math.pow(y, 2);
    double rotSpeed = Math.signum(rot) * Math.pow(rot, 2);

    xSpeed *= SwerveConstants.MAX_ROBOT_TRANS_SPEED;
    ySpeed *= SwerveConstants.MAX_ROBOT_TRANS_SPEED;
    rotSpeed *= SwerveConstants.MAX_ROBOT_TRANS_SPEED;
    
    swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed), swerve.getRotation()));
        
    }

    @Override
    public void end(boolean interrupted) {
        // runs when the command ends
    }



}
