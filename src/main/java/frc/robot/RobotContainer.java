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
import frc.robot.util.AimingTarget;
import frc.robot.util.ObjectiveTracker;
import frc.robot.util.ShooterCalculations;
import frc.robot.commands.Teleop;
import frc.robot.constants.ShooterConstants;
import frc.robot.controllers.Controllers;

import static frc.robot.constants.MiscConstants.isReal;
import static frc.robot.constants.ShooterConstants.DEPOT_TRENCH_SHOOT_VELOCITY;
import static frc.robot.constants.ShooterConstants.HOOD_MIN_ANGLE;
import static frc.robot.constants.ShooterConstants.DEPOT_TRENCH_SHOOT_ANGLE;
import static frc.robot.constants.ShooterConstants.HP_TRENCH_SHOOT_VELOCITY;

import org.littletonrobotics.junction.Logger;

import static frc.robot.constants.ShooterConstants.HP_TRENCH_SHOOT_ANGLE;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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

  public static AimingTarget override = null;
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


    NamedCommands.registerCommand("aimShoot", Commands.parallel(
    hood.setAngleHub(swerve),
    shooter.setHubVelocityCommand(swerve),
    swerve.run(()->{swerve.drive(new ChassisSpeeds(), ShooterCalculations.getYawToTarget(swerve.getPose(), AimingManager.lastestHubShot.aimingTarget()));}),
      new SequentialCommandGroup(
        Commands.waitUntil(() -> {return ShooterCalculations.readyToShoot(swerve.getPose(), hood, shooter, ObjectiveTracker.HUB);}).withTimeout(5),
        spindexer.runSpindexer(12)
      )
    ));
    
    initializeNamedCommands();

    // hood.setDefaultCommand(new HoodNTControl(hood));
    // deploy.setDefaultCommand(new DeployNTControl(deploy));
    // shooter.setDefaultCommand(new ShooterNTControl(shooter));
    hood.setDefaultCommand(hood.setAngleCommand(HOOD_MIN_ANGLE));
    shooter.setDefaultCommand(shooter.run(()->{shooter.setShooterVelocity(0);}));
    deploy.setDefaultCommand(deploy.holdDownCommand());
    intake.setDefaultCommand(intake.run(()->{intake.stop();}));

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
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  


 
  public void configureBindings() {
     if (!Controllers.didControllersChange())
      return;
    
    // Clear buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Find new controllers
    Controllers.updateActiveControllerInstance();


    Controllers.driverController.getLeftPaddle().whileTrue(deploy.shimmy(intake));
    Controllers.driverController.getLeftPaddle().whileFalse(intake.run(() -> {
      intake.stop();
    }));


  // Controllers.driverController.getRightPaddle().whileTrue(Commands.sequence(
  //   Commands.waitUntil(()->{return ShooterCalculations.isReadyToShoot(swerve, hood, shooter);}),
  //   Commands.run(()-> {
  //     spindexer.setVoltageMainSpinner(-12);
  //     spindexer.setVoltageFeeder(12);
  //   }).alongWith(deploy.shimmy(intake))  
  //   ));

  //   Controllers.driverController.getRightPaddle().onFalse(
  //     Commands.runOnce(()-> {
  //       spindexer.setVoltageMainSpinner(0);
  //       spindexer.setVoltageFeeder(0);
  //     })   
  //   );

    Controllers.driverController.getLeftBumper().whileTrue(intake.runEnd(()->{intake.setIntakeVoltage(12);}, ()->{intake.setIntakeVoltage(0);}));

    // todo fix after merge
    // Controllers.driverController.getPovRight().onTrue(Commands.run(()->{intake.setAngle(Rotation2d.fromDegrees(30));}).until(()->{return intake.atAngle(Rotation2d.fromDegrees(30));}));
    // Controllers.driverController.getPovDown().onTrue(Commands.run(()->{intake.setAngle(Rotation2d.fromDegrees(0));}).until(()->{return intake.atAngle(Rotation2d.fromDegrees(0));}));
    // Controllers.driverController.getPovUp().onTrue(Commands.run(()->{intake.setAngle(DEPLOY_MAX_ANGLE);}).until(()->{return intake.atAngle(DEPLOY_MAX_ANGLE);}));
        
    // Controllers.driverController.getXBtn().whileTrue(Commands.run(() -> {
    //   spindexer.setVoltageMainSpinner(-12);
    // }).finallyDo(() ->{spindexer.setVoltageMainSpinner(0);}));

    // Controllers.driverController.getXBtn().whileTrue(Commands.run(() -> {
    //   spindexer.setVoltageFeeder(12);
    // }));
    // Controllers.driverController.getXBtn().whileFalse(Commands.run(() -> {
    //   spindexer.setVoltageFeeder(0);
    // }));

    Controllers.driverController.getXBtn().whileTrue(spindexer.runWhenReady());
    Controllers.driverController.getXBtn().whileTrue(deploy.shimmy(intake));

    Controllers.driverController.getPovUp().whileTrue(Commands.run(()->{
      shooter.setShooterVelocity(HP_TRENCH_SHOOT_VELOCITY);
      hood.setAngle(HP_TRENCH_SHOOT_ANGLE);

    }, shooter, hood));

    Controllers.driverController.getPovLeft().onTrue(Commands.runOnce(()->{
      ShooterConstants.HUB_ANGLE_ADJUSTMENT = ShooterConstants.HUB_ANGLE_ADJUSTMENT.minus(Rotation2d.fromDegrees(0.5));
    }));

    Controllers.driverController.getPovRight().onTrue(Commands.runOnce(()->{
      ShooterConstants.HUB_ANGLE_ADJUSTMENT = ShooterConstants.HUB_ANGLE_ADJUSTMENT.plus(Rotation2d.fromDegrees(0.5));
    }));

    Controllers.driverController.getPovDown().onTrue(hood.setAngleCommand(Rotation2d.kZero));
    
    // Controllers.driverController.getPovDown().whileTrue(Commands.run(()-> {
    //   shooter.setKickerVoltage(-6);
    //   shooter.setFlywheelVoltage(6);
    //   spindexer.setVoltageFeeder(-6);
    // }).finallyDo(() -> {
    //   shooter.setKickerVoltage(0);
    //   shooter.setFlywheelVoltage(0);
    //   spindexer.setVoltageFeeder(0);
    // }));
    // Controllers.driverController.getPovUp().onTrue(hood.hoodHome());
    // Controllers.driverController.getYBtn().whileTrue(deploy.setVoltageCommand(2));
    // Controllers.driverController.getBBtn().whileTrue(deploy.setVoltageCommand(-2));


    Controllers.driverController.getBackButton().onTrue(hood.hoodHome());
    Controllers.driverController.getStartButton().onTrue(Commands.runOnce(()->{
      ShooterCalculations.HubDists.add(ShooterCalculations.getDistanceToTarget(swerve.getPose(), ObjectiveTracker.HUB.getFieldPosition()));
      ShooterCalculations.ShooterSpeeds.add(shooter.getFlywheelVelocityRadPerSec());
      ShooterCalculations.HoodAngles.add(hood.getAngle());
      double[] HubDistsArray = ShooterCalculations.HubDists.stream().mapToDouble(Double::doubleValue).toArray();
      double[] ShooterSpeedsArray = ShooterCalculations.ShooterSpeeds.stream().mapToDouble(Double::doubleValue).toArray();
      double[] HoodAnglesArray = ShooterCalculations.HoodAngles.stream()
                        .mapToDouble(r -> r.getRadians())
                        .toArray();
      Logger.recordOutput("ShooterCalculations/HubDistances", HubDistsArray);
      Logger.recordOutput("ShooterCalculations/ShooterSpeeds", ShooterSpeedsArray);
      Logger.recordOutput("ShooterCalculations/HoodAngles", HoodAnglesArray);
    }));
    // Controllers.driverController.getStartButton().whileTrue(Commands.run(() -> {
    //   intake.setIntakeVoltage(-12);
    // }));
   // Controllers.driverController.getYBtn().whileTrue(Commands.run(() -> {
    //  intake.setDeployVoltage(2);
    //}).finallyDo(() ->{intake.setDeployVoltage(0);}));
    //Controllers.driverController.getBBtn().whileTrue(Commands.run(() -> {
    //   intake.setDeployVoltage(-2);
    // }).finallyDo(() ->{intake.setDeployVoltage(0);}));

    // Controllers.driverController.getYBtn().whileTrue(climber.run(()->{climber.setClimberVoltage(2);}).finallyDo(()->{climber.setClimberVoltage(0);}));
    // Controllers.driverController.getBBtn().whileTrue(climber.run(()->{climber.setClimberVoltage(-2);}).finallyDo(()->{climber.setClimberVoltage(0);}));

    Controllers.driverController.getYBtn().onTrue(climber.climberUp());
    Controllers.driverController.getBBtn().onTrue(climber.climberDown());
    // Controllers.driverController.getYBtn().whileTrue(spindexer.run(()->{
    //   spindexer.runSpindexer(12);
    // }));
    // Controllers.driverController.getYBtn().onTrue(Commands.runOnce(()->{shooter.setShooterVelocity(shooter.getFlywheelSetpoint()+Units.rotationsPerMinuteToRadiansPerSecond(25));}));
    // Controllers.driverController.getBBtn().onTrue(Commands.runOnce(()->{shooter.setShooterVelocity(shooter.getFlywheelSetpoint()-Units.rotationsPerMinuteToRadiansPerSecond(25));}));
    // Controllers.driverController.getPovRight().onTrue(Commands.runOnce(()->{hood.setAngle(Rotation2d.fromRadians(hood.getAngle().getRadians()+Units.degreesToRadians(0.5)));}));
    // Controllers.driverController.getPovLeft().onTrue(Commands.runOnce(()->{hood.setAngle(Rotation2d.fromRadians(hood.getAngle().getRadians()-Units.degreesToRadians(0.5)));}));

    // Controllers.driverController.getRightBumper().whileTrue(shooter.run(()->{
    //   shooter.setShooterVelocity(ShooterCalculations.getHubVelocity(swerve));
    // }));

    // Controllers.driverController.getRightBumper().whileFalse(shooter.run(()->{
    //   shooter.setShooterVelocity(0);
    // }));
    
    Controllers.driverController.getRightTriggerBtn().whileTrue(hood.setAngleFeed(swerve));
    Controllers.driverController.getRightTriggerBtn().whileTrue(shooter.setFeedVelocityCommand(swerve));
    Controllers.driverController.getRightTriggerBtn().whileTrue(Commands.run(()->{
      intake.setIntakeVoltage(12);
      spindexer.setVoltageMainSpinner(-12);
      spindexer.setVoltageFeeder(12);
    }));
    Controllers.driverController.getRightTriggerBtn().onFalse(Commands.runOnce(()->{
      intake.setIntakeVoltage(0);
      spindexer.setVoltageMainSpinner(0);
      spindexer.setVoltageFeeder(0);
    }));

    Controllers.operatorController.getAButton().onTrue(Commands.runOnce(()->{
      ShooterConstants.HUB_ANGLE_ADJUSTMENT = ShooterConstants.HUB_ANGLE_ADJUSTMENT.minus(Rotation2d.fromDegrees(0.5));
    }));

    Controllers.operatorController.getXButton().onTrue(Commands.runOnce(()->{
      ShooterConstants.HUB_ANGLE_ADJUSTMENT = ShooterConstants.HUB_ANGLE_ADJUSTMENT.plus(Rotation2d.fromDegrees(0.5));
    }));
    Controllers.operatorController.getBButton().onTrue(Commands.runOnce(()->{
      ShooterConstants.SHOOTER_RPM_ADJUSTMENT = (ShooterConstants.SHOOTER_RPM_ADJUSTMENT - 50); 
    }));
       Controllers.operatorController.getYButton().onTrue(Commands.runOnce(()->{
      ShooterConstants.SHOOTER_RPM_ADJUSTMENT = (ShooterConstants.SHOOTER_RPM_ADJUSTMENT + 50); 
    }));

    Controllers.operatorController.getRightBumper().whileTrue(climber.run(()->{climber.setClimberVoltage(2);}).finallyDo(()->{climber.setClimberVoltage(0);}));
    Controllers.operatorController.getLeftBumper().whileTrue(climber.run(()->{climber.setClimberVoltage(-2);}).finallyDo(()->{climber.setClimberVoltage(0);}));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return sysidPicker.get();
    // return hood.NTVoltageCommand();
    // return deploy.shimmy(intake);
    //return Swerve.wheelRadiusCharacterization(swerve);
  }

  public void updateAimingManager() {
    ShooterCalculations.log(swerve.getPose());
    AimingManager.update(swerve.getPose(), swerve.getFieldRelativeSpeeds());
  }

  private void initializeNamedCommands() {
    NamedCommands.registerCommand("StartIntakeChomp", Commands.run(() -> {
      intake.setIntakeVoltage(12);
    }, intake));
    NamedCommands.registerCommand("HoodDown", hood.setAngleCommand(Rotation2d.kZero));
    NamedCommands.registerCommand("StopIntakeChomp", Commands.run(() -> {
      intake.setIntakeVoltage(0);
    }, intake));
    NamedCommands.registerCommand("DeployIntake", deploy.holdDownCommand());
    NamedCommands.registerCommand("STOP", Commands.run(() -> {
      swerve.stop();
    }, swerve));
    NamedCommands.registerCommand("FireFromDepotTrench", Commands.run(() -> {
      shooter.setShooterVelocity(Units.rotationsPerMinuteToRadiansPerSecond(DEPOT_TRENCH_SHOOT_VELOCITY));
      hood.setAngle(DEPOT_TRENCH_SHOOT_ANGLE);
    }, shooter, hood));
    NamedCommands.registerCommand("FireFromHPTrench", Commands.run(() -> {
      shooter.setShooterVelocity(Units.rotationsPerMinuteToRadiansPerSecond(HP_TRENCH_SHOOT_VELOCITY));
      hood.setAngle(HP_TRENCH_SHOOT_ANGLE);
    }, shooter, hood));
    NamedCommands.registerCommand("Shimmy", deploy.shimmy(intake));
    NamedCommands.registerCommand("StartSpindexFeed", Commands.run(() -> {
      spindexer.setVoltageMainSpinner(-12);
      spindexer.setVoltageFeeder(12);
    }, spindexer));
    NamedCommands.registerCommand("StopSpindexFeed", Commands.run(() -> {
      spindexer.setVoltageMainSpinner(0);
      spindexer.setVoltageFeeder(0);
    }, spindexer));
    NamedCommands.registerCommand("ClimbUp", climber.climberUp());
    NamedCommands.registerCommand("ClimbDown", climber.climberDown());
  }
}
