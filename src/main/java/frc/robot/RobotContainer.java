// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Teleop;
import frc.robot.constants.MiscConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.controllers.Controllers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
 
  private final Swerve swerve;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve = new Swerve();
    swerve.setDefaultCommand(new Teleop(swerve));
    // Configure the trigger bindings
    configureBindings();
    
  }

 
  public void configureBindings() {
     if (!Controllers.didControllersChange())
      return;
    
    // Clear buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Find new controllers
    Controllers.updateActiveControllerInstance();
  }

  public Command getAutonomousCommand() {
   return Commands.none();

  }
}
