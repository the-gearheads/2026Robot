// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Teleop;
import frc.robot.constants.RobotContants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.FuelSim;
import frc.robot.controllers.Controllers;

import static frc.robot.constants.MiscConstants.isReal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
 
  private final Swerve swerve;
  private final Shooter shooter;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve = new Swerve();
    swerve.setDefaultCommand(new Teleop(swerve));
    shooter = new Shooter();
    // Configure the trigger bindings
    configureBindings();
    if (!isReal) {
      FuelSim.getInstance().spawnStartingFuel(); // spawns fuel in the depots and neutral zone

      FuelSim.getInstance().registerRobot(
        RobotContants.ROBOT_WIDTH,  // from left to right
        RobotContants.ROBOT_LENGTH, // from front to back
        RobotContants.ROBOT_BUMPER_HEIGHT,// from floor to top of bumpers
        swerve::getPose, // Supplier<Pose2d> of robot pose
        swerve::getFieldRelativeSpeeds); // Supplier<ChassisSpeeds> of field-centric chassis speeds
      
      FuelSim.getInstance().start();
    }
  }

 
  public void configureBindings() {
     if (!Controllers.didControllersChange())
      return;
    
    // Clear buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Find new controllers
    Controllers.updateActiveControllerInstance();

    Controllers.driverController.getABtn().whileTrue(shooter.runShooter(12));
    Controllers.driverController.getBBtn().whileTrue(shooter.runShooter(6));
    Controllers.driverController.getXBtn().whileTrue(shooter.runShooter(9));
  }

  public Command getAutonomousCommand() {
   return Commands.none();

  }
}
