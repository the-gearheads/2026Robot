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
import frc.robot.util.HubTracker;
import frc.robot.util.ObjectiveTracker;
import frc.robot.util.ShooterCalculations;
import frc.robot.util.targets.VirtualTarget;
import frc.robot.commands.Teleop;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.controllers.Controllers;

import static frc.robot.constants.IntakeConstants.DEPLOY_MAX_ANGLE;
import static frc.robot.constants.MiscConstants.isReal;
import static frc.robot.constants.ShooterConstants.DEPOT_TRENCH_SHOOT_VELOCITY;
import static frc.robot.constants.ShooterConstants.HOOD_MIN_ANGLE;
import static frc.robot.constants.ShooterConstants.DEPOT_TRENCH_SHOOT_ANGLE;
import static frc.robot.constants.ShooterConstants.HP_TRENCH_SHOOT_VELOCITY;

import static frc.robot.constants.ShooterConstants.HP_TRENCH_SHOOT_ANGLE;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
    // sysidPicker.addSysidRoutines("Swerve Angular", swerve.getAngularSysIdRoutine());  // we only need this for Choreo
    sysidPicker.addSysidRoutines("Shooter Main Fly", shooter.getMainFlySysidRoutine());
    sysidPicker.addSysidRoutines("Shooter Kicker", shooter.getKickerSysidRoutine());
    sysidPicker.addSysidRoutines("Hood", hood.getSysIdRoutine(), hood::forwardSysIdLimit, hood::reverseSysIdLimit);
    sysidPicker.addSysidRoutines("Feeder", spindexer.getFeederSysidRoutine());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  


 
  public void configureBindings() {
     if (!Controllers.didControllersChange())
      return;
    
    // Clear buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    // CommandScheduler.getInstance().getDefaultButtonLoop().clear();

    // Find new controllers
    Controllers.updateActiveControllerInstance();

    HubTracker.NEXT_ACTIVE_SHIFT_TRIGGER.onTrue(Commands.deferredProxy(() -> {
      if (DriverStation.isTeleopEnabled()) {
        return Controllers.driverController.getRumbleCommand(1, 0.2, 3);
      } else {
        return Commands.none();
      }
    }));

    HubTracker.NEXT_SHIFT_INACTIVE_TRIGGER.onTrue(Commands.deferredProxy(() -> {
      if (DriverStation.isTeleopEnabled()) {
        return Controllers.driverController.getRumbleCommand(1, 1);
      } else {
        return Commands.none();
      }
    }));

    Controllers.driverController.getRightTriggerBtn().whileTrue(Commands.parallel(
        hood.setObjectiveAngleCommand(swerve),
        shooter.setObjectiveVelocityCommand(swerve),
        new SequentialCommandGroup(
          Commands.waitUntil(() -> {return ShooterCalculations.readyToShoot(swerve.getPose(), hood, shooter);}).withTimeout(5),
          Commands.deferredProxy(()->{
             if( //ShooterCalculations.isTimeToShoot(AimingManager.latestShot.timeOfFlight()) &&
              AimingManager.latestShot.aimingTarget() == ObjectiveTracker.HUB || (AimingManager.latestShot.aimingTarget() instanceof VirtualTarget) && ((VirtualTarget)AimingManager.latestShot.aimingTarget()).baseTarget == ObjectiveTracker.HUB) {
              if (swerve.getSpeedMagnitude() > SwerveConstants.SHIMMY_THRESHOLD_SPEED) {
                return spindexer.runSpindexer(12);
              } else {
                return spindexer.runSpindexer(12);//.alongWith(deploy.shimmy(intake));
              }
            } else {
              return spindexer.runSpindexer(12);
            }
          })
        )
      )
    );

    Controllers.driverController.getRightBumper().whileTrue(Commands.parallel(
        hood.setAngleHub(swerve),
        shooter.setHubVelocityCommand(swerve),
        new SequentialCommandGroup(
          Commands.waitUntil(() -> {return ShooterCalculations.readyToShoot(swerve.getPose(), hood, shooter, ObjectiveTracker.HUB);}),
          Commands.deferredProxy(()->{
            if (swerve.getSpeedMagnitude() > SwerveConstants.SHIMMY_THRESHOLD_SPEED) {
                return spindexer.runSpindexer(12);
              } else {
                return spindexer.runSpindexer(12);//.alongWith(deploy.shimmy(intake));
              }
            })
          )
      )
    );

    Controllers.driverController.getLeftBumper().whileTrue(Commands.parallel(
      hood.setAngleFeed(swerve),
      shooter.setFeedVelocityCommand(swerve),
      new SequentialCommandGroup(
          Commands.waitUntil(() -> {return ShooterCalculations.readyToShoot(swerve.getPose(), hood, shooter, ObjectiveTracker.getFeedingObjective(swerve.getPose()));}),
          spindexer.runSpindexer(12)
      )
    ));

    Controllers.driverController.getLeftTriggerBtn().whileTrue(intake.run(()->{intake.setIntakeVoltage(12);}));

    Controllers.driverController.getXBtn().whileTrue(spindexer.runSpindexer(12));
    Controllers.driverController.getABtn().whileTrue(deploy.shimmy(intake));
    Controllers.driverController.getYBtn().onTrue(climber.climberUp());
    Controllers.driverController.getBBtn().onTrue(climber.climberDown());

    // Controllers.driverController.getYBtn().whileTrue(spindexer.run(()->{spindexer.setVoltageFloober(12);}).finallyDo(
    //   ()->{spindexer.setVoltageFloober(0);}
    // ));
    // Controllers.driverController.getBBtn().onTrue(climber.autoClimb(swerve));

    Controllers.driverController.getLeftPaddle().whileTrue(Commands.run(() -> {
      shooter.setShooterVelocity(HP_TRENCH_SHOOT_VELOCITY);
      hood.setAngle(HP_TRENCH_SHOOT_ANGLE);
    }, shooter, hood).alongWith(spindexer.runSpindexer(12).alongWith(deploy.shimmy(intake))));

    Controllers.driverController.getPovUp().onTrue(Commands.runOnce(()->{
      ShooterConstants.HOOD_ANGLE_ADJUSTMENT = ShooterConstants.HOOD_ANGLE_ADJUSTMENT.minus(Rotation2d.fromDegrees(0.5));
    }));

    Controllers.driverController.getPovDown().onTrue(Commands.runOnce(()->{
      ShooterConstants.HOOD_ANGLE_ADJUSTMENT = ShooterConstants.HOOD_ANGLE_ADJUSTMENT.plus(Rotation2d.fromDegrees(0.5));
    }));

    Controllers.driverController.getPovLeft().whileTrue(Commands.run(
      ()->{
        climber.setPosition(ClimberConstants.MIN_CLIMBER_POS + 1);
        climber.setClimberVoltage(-3);
      }, climber).finallyDo(()->{climber.setClimberVoltage(0);}));

    Controllers.driverController.getPovRight().whileTrue(Commands.run(
      ()->{
        climber.setPosition(ClimberConstants.MAX_CLIMBER_POS -1);
        climber.setClimberVoltage(3);
      }, climber).finallyDo(()->{climber.setClimberVoltage(0);}));


    Controllers.driverController.getBackButton().onTrue(hood.hoodHome());
    Controllers.driverController.getRightPaddle().whileTrue(intake.run((()->{intake.setIntakeVoltage(-12);})).finallyDo(()->{
      intake.setIntakeVoltage(0);
    }));
    Controllers.driverController.getStartButton().onTrue(Commands.runOnce(() -> {swerve.waitToCrossToggle();}));
    // Controllers.driverController.getStartButton().onTrue(Commands.runOnce(()->{
    //   ShooterCalculations.HubDists.add(ShooterCalculations.getDistanceToTarget(swerve.getPose(), ObjectiveTracker.HUB.getFieldPosition()));
    //   ShooterCalculations.ShooterSpeeds.add(shooter.getFlywheelVelocityRadPerSec());
    //   ShooterCalculations.HoodAngles.add(hood.getAngle());
    //   double[] HubDistsArray = ShooterCalculations.HubDists.stream().mapToDouble(Double::doubleValue).toArray();
    //   double[] ShooterSpeedsArray = ShooterCalculations.ShooterSpeeds.stream().mapToDouble(Double::doubleValue).toArray();
    //   double[] HoodAnglesArray = ShooterCalculations.HoodAngles.stream()
    //                     .mapToDouble(r -> r.getRadians())
    //                     .toArray();
    //   Logger.recordOutput("ShooterCalculations/HubDistances", HubDistsArray);
    //   Logger.recordOutput("ShooterCalculations/ShooterSpeeds", ShooterSpeedsArray);
    //   Logger.recordOutput("ShooterCalculations/HoodAngles", HoodAnglesArray);
    // }));


    //  Controllers.operatorController.getAButton().onTrue(Commands.runOnce(()->{
    //    ShooterConstants.HOOD_ANGLE_ADJUSTMENT = ShooterConstants.HOOD_ANGLE_ADJUSTMENT.minus(Rotation2d.fromDegrees(1));
    //  }));

    // Controllers.operatorController.getXButton().onTrue(Commands.runOnce(()->{
    //   ShooterConstants.HOOD_ANGLE_ADJUSTMENT = ShooterConstants.HOOD_ANGLE_ADJUSTMENT.plus(Rotation2d.fromDegrees(1));
    // }));
    // Controllers.operatorController.getBButton().onTrue(Commands.runOnce(()->{
    //   ShooterConstants.SHOOTER_VEL_ADJUSTMENT = (ShooterConstants.SHOOTER_VEL_ADJUSTMENT - 50); 
    // }));
    //    Controllers.operatorController.getYButton().onTrue(Commands.runOnce(()->{
    //   ShooterConstants.SHOOTER_VEL_ADJUSTMENT = (ShooterConstants.SHOOTER_VEL_ADJUSTMENT + 50);
    // }));

    Controllers.operatorController.getAButton().whileTrue(deploy.shimmy(intake));
    Controllers.operatorController.getBButton().whileTrue(intake.run((()->{intake.setIntakeVoltage(-12);})).finallyDo(()->{
      intake.setIntakeVoltage(0);
    }));
    Controllers.operatorController.getCButton().whileTrue(deploy.run(()->{deploy.setAngle(DEPLOY_MAX_ANGLE);}));

    Controllers.operatorController.getRightBumper().whileTrue(climber.run(()->{climber.setClimberVoltage(2);}).finallyDo(()->{climber.setClimberVoltage(0);}));
    Controllers.operatorController.getLeftBumper().whileTrue(climber.run(()->{climber.setClimberVoltage(-2);}).finallyDo(()->{climber.setClimberVoltage(0);}));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return sysidPicker.get();
    //return Swerve.wheelRadiusCharacterization(swerve);
  }

  public void updateAimingManager() {
    ShooterCalculations.log(swerve.getPose());
    AimingManager.update(swerve.getPose(), swerve.getFieldRelativeSpeeds(),  swerve.getGyroXAcceleration(), swerve.getGyroYAcceleration(), swerve.getGyroYawAcceleration());
  }

  private void initializeNamedCommands() {
    NamedCommands.registerCommand("aimShoot", Commands.parallel(
        hood.setAngleHub(swerve),
        shooter.setHubVelocityCommand(swerve),
        swerve.run(() -> {
          swerve.drive(new ChassisSpeeds(),
              ShooterCalculations.getYawToTarget(swerve.getPose(), AimingManager.latestHubShot.aimingTarget()));
        }),
        new SequentialCommandGroup(
            Commands.waitUntil(() -> {
              return ShooterCalculations.readyToShoot(swerve.getPose(), hood, shooter, ObjectiveTracker.HUB);
            }).withTimeout(5),
            spindexer.runSpindexer(12).alongWith(deploy.shimmy(intake)))));
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
    NamedCommands.registerCommand("TrenchShoot", Commands.run(() -> {
      shooter.setShooterVelocity(HP_TRENCH_SHOOT_VELOCITY);
      hood.setAngle(HP_TRENCH_SHOOT_ANGLE);
    }, shooter, hood).alongWith(spindexer.runSpindexer(12).alongWith(deploy.shimmy(intake))));
     NamedCommands.registerCommand("aimShootStop", Commands.parallel(
        hood.setAngleHub(swerve),
        shooter.setHubVelocityCommand(swerve),
        swerve.run(() -> {
          swerve.drive(new ChassisSpeeds(),
              ShooterCalculations.getYawToTarget(swerve.getPose(), AimingManager.latestHubShot.aimingTarget()));
        }),
        new SequentialCommandGroup(
            Commands.waitUntil(() -> {
              return ShooterCalculations.readyToShoot(swerve.getPose(), hood, shooter, ObjectiveTracker.HUB);
            }).withTimeout(5),
            spindexer.runSpindexer(12).alongWith(deploy.shimmy(intake)))).until(shooter::shouldIGo));
      NamedCommands.registerCommand( "AutoClimb", climber.autoClimb(swerve));
  }
}
