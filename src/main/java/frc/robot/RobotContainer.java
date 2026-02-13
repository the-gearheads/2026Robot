// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Teleop;
import frc.robot.constants.RobotContants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.HoodSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.FuelSim;
import frc.robot.controllers.Controllers;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.MiscConstants.isReal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
 
  private final Swerve swerve;
  private final SysidAutoPicker sysidPicker;
  private final Spindexer spindexer;
  private final Hood hood;
  private final Shooter shooter;
  private final Intake intake;

  public FuelSim fuelSim = new FuelSim("FuelSim"); // creates a new fuelSim of FuelSim
  //public Dog dog = new Dog;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    sysidPicker = new SysidAutoPicker();
    swerve = new Swerve();
    swerve.setDefaultCommand(new Teleop(swerve));
    // Configure the trigger bindings
    
    if (!isReal) {
      spindexer = new SpindexerSim();
      hood = new HoodSim();
      shooter = new ShooterSim();
      intake = new IntakeSim();
      configureFuelSim();
    } else {
      spindexer = new Spindexer();
      hood = new Hood();
      shooter = new Shooter();
      intake = new Intake();
    }

    configureBindings();
    sysidPicker.addSysidRoutines("Swerve Drive", swerve.getDriveSysIdRoutine());
    // sysidPicker.addSysidRoutines("Swerve Angular", swerve.getAngularSysIdRoutine());  // we only need this for Choreo
    sysidPicker.addSysidRoutines("Shooter Main Fly", shooter.getMainFlySysidRoutine());
    sysidPicker.addSysidRoutines("Shooter Kicker", shooter.getKickerSysidRoutine());
    sysidPicker.addSysidRoutines("Hood", hood.getSysIdRoutine());
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
    Controllers.driverController.getRightBumper().onTrue(Commands.runOnce(() -> {
      fuelSim.launchFuel(MetersPerSecond.of(shooter.getFlywheelVelocityRadPerSec() * ShooterConstants.FLYWHEEL_RADIUS),
          hood.getAngle().getMeasure(),
          Rotation2d.kZero.getMeasure(), Inches.of(22));
    }, shooter));
    // Controllers.driverController.getBBtn().whileTrue(shooter.runShooter(6));
    // Controllers.driverController.getXBtn().whileTrue(shooter.runShooter(9));
    Controllers.driverController.getRightTriggerBtn().whileTrue(hood.hoodManual(3));
    Controllers.driverController.getLeftTriggerBtn().whileTrue(hood.hoodManual(-3));
  }


  public Command getAutonomousCommand() {
    return sysidPicker.get();
  }

  private void configureFuelSim() {
    fuelSim.spawnStartingFuel();

    fuelSim.registerRobot(RobotContants.ROBOT_WIDTH,  // from left to right
        RobotContants.ROBOT_LENGTH, // from front to back
        RobotContants.ROBOT_BUMPER_HEIGHT,// from floor to top of bumpers
        swerve::getPose, // Supplier<Pose2d> of robot pose
        swerve::getFieldRelativeSpeeds); // Supplier<ChassisSpeeds> of field-centric chassis speeds
    
    fuelSim.start();
  }
}
