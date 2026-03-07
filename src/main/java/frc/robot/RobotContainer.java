// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.HoodSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.commands.Teleop;
import frc.robot.commands.NTControl.HoodNTControl;
import frc.robot.controllers.Controllers;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.MiscConstants.isReal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotController.RadioLEDState;
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

  public static final boolean deathMode = true;

  // public FuelSim fuelSim = new FuelSim("FuelSim"); // creates a new fuelSim of FuelSim
  // public Dog dog = new Dog;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    if(deathMode) {
      Commands.repeatingSequence(
        Commands.runOnce(()->{RobotController.setRadioLEDState(RadioLEDState.kRed);}),
        Commands.waitSeconds(0.3),
        Commands.runOnce(()->{RobotController.setRadioLEDState(RadioLEDState.kOrange);}),
        Commands.waitSeconds(0.3)
      ).schedule();
    }

    sysidPicker = new SysidAutoPicker();

    swerve = new Swerve();
    swerve.setDefaultCommand(new Teleop(swerve));
    // Configure the trigger bindings
    if (!isReal) {
      spindexer = new SpindexerSim();
      hood = new HoodSim();
      shooter = new ShooterSim();
      intake = new IntakeSim();
      // configureFuelSim();
    } else {
      spindexer = new Spindexer();
      hood = new Hood();
      shooter = new Shooter();
      intake = new Intake();
    }

    hood.setDefaultCommand(new HoodNTControl(hood));

    configureBindings();
    sysidPicker.addSysidRoutines("Swerve Drive", swerve.getDriveSysIdRoutine());
    sysidPicker.addSysidRoutines("Intake Deploy", intake.getDeploySysid(), intake::getForwardSysidLimit, intake::getBackwardSysidLimit);
    // // sysidPicker.addSysidRoutines("Swerve Angular", swerve.getAngularSysIdRoutine());  // we only need this for Choreo
    // sysidPicker.addSysidRoutines("Shooter Main Fly", shooter.getMainFlySysidRoutine());
    // sysidPicker.addSysidRoutines("Shooter Kicker", shooter.getKickerSysidRoutine());
    sysidPicker.addSysidRoutines("Hood", hood.getSysIdRoutine(), hood::forwardSysIdLimit, hood::reverseSysIdLimit);
    sysidPicker.addSysidRoutines("Feeder", spindexer.getFeederSysidRoutine());

    // hood.setDefaultCommand(hood.run(() -> {
    //   hood.setAngle(ShooterCalculations.getHubAngle(swerve.getPose()));
    // }));

    // shooter.setDefaultCommand(shooter.run(() -> {
    //   shooter.setFlywheelVelocity(ShooterCalculations.getHubDistance(swerve.getPose()));
    //   shooter.setKickerVelocity(ShooterCalculations.getHubVelocity(swerve.getPose()));
    // }));
  }
  


 
  public void configureBindings() {
     if (!Controllers.didControllersChange())
      return;
    
    // Clear buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Find new controllers
    Controllers.updateActiveControllerInstance();


    // voltage numbers are completely arbitrary ngl i just picked things
    Controllers.driverController.getABtn().whileTrue(shooter.run(()->{
      shooter.setFlywheelVelocity(Units.rotationsPerMinuteToRadiansPerSecond(-4000));
      shooter.setKickerVelocity(Units.rotationsPerMinuteToRadiansPerSecond(4000));
    }).finallyDo(()->{
      shooter.setFlywheelVoltage(0);
      shooter.setKickerVoltage(0);
    }));

    Controllers.driverController.getLeftPaddle().whileTrue(hood.run(()->{
      hood.setAngle(Rotation2d.fromDegrees(20));
    }).andThen(hood.run(()->{hood.setVoltage(0);})));

    // Controllers.driverController.getRightBumper().onTrue(Commands.runOnce(() -> {
    //   fuelSim.launchFuel(MetersPerSecond.of(shooter.getFlywheelVelocityRadPerSec() * ShooterConstants.FLYWHEEL_RADIUS),
    //       hood.getAngle().getMeasure(),
    //       Rotation2d.kZero.getMeasure(), Inches.of(22));
    // }, shooter));

    Controllers.driverController.getRightTriggerBtn().whileTrue(hood.hoodManual(3));
    Controllers.driverController.getLeftTriggerBtn().whileTrue(hood.hoodManual(-3));
    Controllers.driverController.getLeftBumper().whileTrue(intake.runEnd(()->{intake.setIntakeVoltage(12);}, ()->{intake.setIntakeVoltage(0);}));

    Controllers.driverController.getXBtn().whileTrue(Commands.run(() -> {
      spindexer.setVoltageMainSpinner(-12);
    }).finallyDo(() ->{spindexer.setVoltageMainSpinner(0);}));

    Controllers.driverController.getXBtn().whileTrue(Commands.run(() -> {
      spindexer.setFeederSpeed(Units.rotationsPerMinuteToRadiansPerSecond(4000));
    }));

    Controllers.driverController.getXBtn().whileFalse(Commands.run(() -> {
      spindexer.setVoltageFeeder(0);
    }));
    
    Controllers.driverController.getPovDown().whileTrue(Commands.run(()-> {
      // shooter.setKickerVoltage(-6);
      shooter.setFlywheelVoltage(6);
      // spindexer.setVoltageFeeder(-6);
    }).finallyDo(() -> {
      // shooter.setKickerVoltage(0);
      shooter.setFlywheelVoltage(0);
      // spindexer.setVoltageFeeder(0);
    }));

    Controllers.driverController.getPovUp().onTrue(hood.hoodHome(-2));
    Controllers.driverController.getYBtn().whileTrue(Commands.run(() -> {
      intake.setDeployVoltage(2);
    }).finallyDo(() ->{intake.setDeployVoltage(0);}));
    Controllers.driverController.getBBtn().whileTrue(Commands.run(() -> {
      intake.setDeployVoltage(-2);
    }).finallyDo(() ->{intake.setDeployVoltage(0);}));

    // Controllers.driverController.getXBtn().whileTrue(spindexer.run(()->{
    //   spindexer.setVoltageFeeder(12);
    // }));
  }

  public Command getAutonomousCommand() {
    return sysidPicker.get();
    // return intake.run(() -> {
    //   intake.setAngle(Rotation2d.fromDegrees(10));
    // });
    // return Commands.none();
  }

  // private void configureFuelSim() {
  //   fuelSim.spawnStartingFuel();

  //   fuelSim.registerRobot(RobotContants.ROBOT_WIDTH,  // from left to right
  //       RobotContants.ROBOT_LENGTH, // from front to back
  //       RobotContants.ROBOT_BUMPER_HEIGHT,// from floor to top of bumpers
  //       swerve::getPose, // Supplier<Pose2d> of robot pose
  //       swerve::getFieldRelativeSpeeds); // Supplier<ChassisSpeeds> of field-centric chassis speeds
    
  //   fuelSim.start();
  // }
}
