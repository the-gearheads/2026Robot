// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Teleop;
import frc.robot.constants.RobotContants;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.HoodSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.FuelSim;
import frc.robot.controllers.Controllers;

import static frc.robot.constants.MiscConstants.isReal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotContainer {
 
  private final Swerve swerve;
  private final Shooter shooter;
  private final Hood hood;
  private final SysidAutoPicker sysidPicker;
  private final Spindexer spindexer;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    sysidPicker = new SysidAutoPicker();
    swerve = new Swerve();
    swerve.setDefaultCommand(new Teleop(swerve));
    // Configure the trigger bindings
    
    if (!isReal) {

      shooter = new ShooterSim();
      hood = new HoodSim();
      spindexer = new SpindexerSim();
      FuelSim.getInstance().spawnStartingFuel(); // spawns fuel in the depots and neutral zone

      FuelSim.getInstance().registerRobot(
        RobotContants.ROBOT_WIDTH,  // from left to right
        RobotContants.ROBOT_LENGTH, // from front to back
        RobotContants.ROBOT_BUMPER_HEIGHT,// from floor to top of bumpers
        swerve::getPose, // Supplier<Pose2d> of robot pose
        swerve::getFieldRelativeSpeeds); // Supplier<ChassisSpeeds> of field-centric chassis speeds
      
      FuelSim.getInstance().start();
    } else {
      shooter = new Shooter();
      hood = new Hood();
      spindexer = new Spindexer();
    }
    configureBindings();

    sysidPicker.addSysidRoutines("main Shooter", shooter.getMainFlySysidRoutine());
    sysidPicker.addSysidRoutines("top Shooter", shooter.getKickerSysidRoutine());
    sysidPicker.addSysidRoutines("swerve drive", swerve.getDriveSysIdRoutine());
    sysidPicker.addSysidRoutines("hood", hood.getSysIdRoutine(), hood::forwardSysIdLimit, hood::reverseSysIdLimit);
  }

 
  public void configureBindings() {
     if (!Controllers.didControllersChange())
      return;
    
    // Clear buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Find new controllers
    Controllers.updateActiveControllerInstance();


    // voltage numbers are completely arbitrary ngl i just picked things
    Controllers.driverController.getABtn().whileTrue(shooter.runShooter(12));
    // Controllers.driverController.getBBtn().whileTrue(shooter.runShooter(6));
    // Controllers.driverController.getXBtn().whileTrue(shooter.runShooter(9));

    Controllers.driverController.getLeftBumper().whileTrue(hood.hoodManual(3));
    Controllers.driverController.getRightBumper().whileTrue(hood.hoodManual(-3));
  }


  public Command getAutonomousCommand() {
    return sysidPicker.get();
  }
}
