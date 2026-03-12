// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberSim;
import frc.robot.subsystems.intake.Deploy;
import frc.robot.subsystems.intake.DeploySim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.HoodSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ShooterCalculations;
import frc.robot.commands.Teleop;
import frc.robot.commands.NTControl.HoodNTControl;
import frc.robot.commands.NTControl.ShooterNTControl;
import frc.robot.commands.NTControl.DeployNTControl;
import frc.robot.controllers.Controllers;

import static frc.robot.constants.IntakeConstants.DEPLOY_MIN_ANGLE;
import static frc.robot.constants.MiscConstants.isReal;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final Climber climber;
  private final SendableChooser<Command> autoChooser;
  private final Deploy deploy;

  public static final boolean deathMode = true;

  // public FuelSim fuelSim = new FuelSim("FuelSim"); // creates a new fuelSim of FuelSim
  // public Dog dog = new Dog;
  
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
      climber = new ClimberSim();
      deploy = new DeploySim();
      // configureFuelSim();
    } else {
      spindexer = new Spindexer();
      hood = new Hood();
      shooter = new Shooter();
      intake = new Intake();
      climber = new Climber();
      deploy = new Deploy();
    }

    swerve.setPose(AllianceFlipUtil.apply(new Pose2d(3.560, 4.025, Rotation2d.k180deg)));

    hood.setDefaultCommand(new HoodNTControl(hood));
    shooter.setDefaultCommand(new ShooterNTControl(shooter));
    deploy.setDefaultCommand(new DeployNTControl(deploy));

    configureBindings();
    sysidPicker.addSysidRoutines("Swerve Drive", swerve.getDriveSysIdRoutine());
    sysidPicker.addSysidRoutines("Intake Deploy", deploy.getDeploySysid(), deploy::getForwardSysidLimit, deploy::getBackwardSysidLimit);
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
     // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
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
      shooter.setFlywheelVelocity(Units.rotationsPerMinuteToRadiansPerSecond(2500));
    }).finallyDo(()->{
      shooter.setFlywheelVoltage(0);
      shooter.setKickerVoltage(0);
    }));

    Controllers.driverController.getLeftPaddle().whileTrue(deploy.shimmy(intake));
    deploy.setDefaultCommand(deploy.setAngleCommand(DEPLOY_MIN_ANGLE));
    Controllers.driverController.getLeftPaddle().whileFalse(intake.run(() -> {
      intake.stopIntake();
    }));

    // Controllers.driverController.getLeftPaddle().onTrue(hood.setAngleCommand(Rotation2d.fromDegrees(30)));
    // Controllers.driverController.getRightPaddle().onTrue(hood.setAngleCommand(Rotation2d.fromDegrees(5)));
    // Controllers.driverController.getBBtn().whileTrue(hood.run(()-> {hood.setAngle(Rotation2d.fromDegrees(20));}));
    // Controllers.driverController.getRightBumper().onTrue(Commands.runOnce(() -> {
    //   fuelSim.launchFuel(MetersPerSecond.of(shooter.getFlywheelVelocityRadPerSec() * ShooterConstants.FLYWHEEL_RADIUS),
    //       hood.getAngle().getMeasure(),
    //       Rotation2d.kZero.getMeasure(), Inches.of(22));
    // }, shooter));

    Controllers.driverController.getRightTriggerBtn().whileTrue(hood.hoodManual(3));
    Controllers.driverController.getLeftTriggerBtn().whileTrue(hood.hoodManual(-3));
    Controllers.driverController.getLeftBumper().whileTrue(intake.runEnd(()->{intake.setIntakeVoltage(12);}, ()->{intake.setIntakeVoltage(0);}));

    // todo fix after merge
    // Controllers.driverController.getPovRight().onTrue(Commands.run(()->{intake.setAngle(Rotation2d.fromDegrees(30));}).until(()->{return intake.atAngle(Rotation2d.fromDegrees(30));}));
    // Controllers.driverController.getPovDown().onTrue(Commands.run(()->{intake.setAngle(Rotation2d.fromDegrees(0));}).until(()->{return intake.atAngle(Rotation2d.fromDegrees(0));}));
    // Controllers.driverController.getPovUp().onTrue(Commands.run(()->{intake.setAngle(DEPLOY_MAX_ANGLE);}).until(()->{return intake.atAngle(DEPLOY_MAX_ANGLE);}));
        
    Controllers.driverController.getXBtn().whileTrue(Commands.run(() -> {
      spindexer.setVoltageMainSpinner(-12);
    }).finallyDo(() ->{spindexer.setVoltageMainSpinner(0);}));

    Controllers.driverController.getXBtn().whileTrue(Commands.run(() -> {
      spindexer.setVoltageFeeder(12);
    }));

    Controllers.driverController.getXBtn().whileFalse(Commands.run(() -> {
      spindexer.setVoltageFeeder(0);
    }));
    
    // Controllers.driverController.getPovDown().whileTrue(Commands.run(()-> {
    //   shooter.setKickerVoltage(-6);
    //   shooter.setFlywheelVoltage(6);
    //   spindexer.setVoltageFeeder(-6);
    // }).finallyDo(() -> {
    //   shooter.setKickerVoltage(0);
    //   shooter.setFlywheelVoltage(0);
    //   spindexer.setVoltageFeeder(0);
    // }));
    Controllers.driverController.getPovUp().onTrue(hood.hoodHome());
    // Controllers.driverController.getYBtn().whileTrue(deploy.setVoltageCommand(2));
    // Controllers.driverController.getBBtn().whileTrue(deploy.setVoltageCommand(-2));


    Controllers.driverController.getBackButton().onTrue(hood.hoodHome());
    Controllers.driverController.getStartButton().onTrue(Commands.runOnce(()->{
      ShooterCalculations.HubDists.add(ShooterCalculations.getHubDistance(swerve.getPose()));
      ShooterCalculations.ShooterSpeeds.add(shooter.getFlywheelVelocityRadPerSec());
      ShooterCalculations.HoodAngles.add(hood.getAngle());
      double[] HubDistsArray = ShooterCalculations.HubDists.stream().mapToDouble(Double::doubleValue).toArray();
      double[] ShooterSpeedsArray = ShooterCalculations.HubDists.stream().mapToDouble(Double::doubleValue).toArray();
      double[] HoodAnglesArray = ShooterCalculations.HoodAngles.stream()
                        .mapToDouble(r -> r.getRadians())
                        .toArray();
      Logger.recordOutput("ShooterCalculations/HubDistances", HubDistsArray);
      Logger.recordOutput("ShooterCalculations/ShooterSpeeds", ShooterSpeedsArray);
      Logger.recordOutput("ShooterCalculations/HoodAngles", HoodAnglesArray);
    }));
   // Controllers.driverController.getYBtn().whileTrue(Commands.run(() -> {
    //  intake.setDeployVoltage(2);
    //}).finallyDo(() ->{intake.setDeployVoltage(0);}));
    //Controllers.driverController.getBBtn().whileTrue(Commands.run(() -> {
    //   intake.setDeployVoltage(-2);
    // }).finallyDo(() ->{intake.setDeployVoltage(0);}));

     Controllers.driverController.getXBtn().whileTrue(spindexer.run(()->{
       spindexer.setVoltageFeeder(12);
     }));
    Controllers.driverController.getYBtn().whileTrue(climber.run(()->{climber.setClimberVoltage(2);}).finallyDo(()->{climber.setClimberVoltage(0);}));
    Controllers.driverController.getBBtn().whileTrue(climber.run(()->{climber.setClimberVoltage(-2);}).finallyDo(()->{climber.setClimberVoltage(0);}));
    // Controllers.driverController.getYBtn().onTrue(climber.climberUp());
    // Controllers.driverController.getBBtn().onTrue(climber.climberDown());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return sysidPicker.get();
    // return Commands.run(()->{hood.setAngle(Rotation2d.fromDegrees(20));});
    // return intake.shimmy();
    //return Swerve.wheelRadiusCharacterization(swerve);
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
