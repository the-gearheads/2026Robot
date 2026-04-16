package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.util.ShooterCalculations;
import frc.robot.AimingManager;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.swerve.Swerve;

public class Shooter extends SubsystemBase {

  SparkFlex mainFly = new SparkFlex(MAIN_FLY_ID, MotorType.kBrushless);
  SparkClosedLoopController flywheelController = mainFly.getClosedLoopController();
  SparkFlexConfig mainFlyConfig = new SparkFlexConfig();
  RelativeEncoder flywheelEncoder = mainFly.getEncoder();

  SparkFlex followerFly = new SparkFlex(FOLLOWER_FLY_ID, MotorType.kBrushless);
  SparkFlexConfig followerFlyConfig = new SparkFlexConfig();

  SparkFlex kicker = new SparkFlex(KICKER_ID, MotorType.kBrushless);
  SparkClosedLoopController kickerController = kicker.getClosedLoopController();
  SparkFlexConfig kickerConfig = new SparkFlexConfig();
  RelativeEncoder kickerEncoder = kicker.getEncoder();

  ShooterCalculations shooterCalculations = new ShooterCalculations();

  public Shooter() {
    configure();
  }

  @Override
  public void periodic() {

  }

  public void configure() {
    mainFlyConfig.smartCurrentLimit(80);
    mainFlyConfig.inverted(true);
    mainFlyConfig.idleMode(IdleMode.kCoast);
    mainFlyConfig.disableVoltageCompensation();
    mainFlyConfig.closedLoop.pid(FLYWHEEL_PID[0] / 12.0, FLYWHEEL_PID[1] / 12.0, FLYWHEEL_PID[2] / 12.0);
    mainFlyConfig.closedLoop.feedForward.kS(FLYWHEEL_FEEDFORWARD.getKs());
    mainFlyConfig.closedLoop.feedForward.kV(FLYWHEEL_FEEDFORWARD.getKv());
    mainFlyConfig.closedLoop.feedForward.kA(FLYWHEEL_FEEDFORWARD.getKa());

    mainFlyConfig.encoder.positionConversionFactor(FLYWHEEL_POS_FACTOR);
    mainFlyConfig.encoder.velocityConversionFactor(FLYWHEEL_VEL_FACTOR);

    mainFlyConfig.signals.primaryEncoderVelocityPeriodMs(5);
    followerFlyConfig.signals.primaryEncoderVelocityPeriodMs(5);
    kickerConfig.signals.primaryEncoderVelocityPeriodMs(5);

    mainFlyConfig.encoder.quadratureMeasurementPeriod(10);
    mainFlyConfig.encoder.quadratureAverageDepth(2); // subject to change
    followerFlyConfig.encoder.quadratureMeasurementPeriod(10);
    followerFlyConfig.encoder.quadratureAverageDepth(2); // subject to change
    kickerConfig.encoder.quadratureMeasurementPeriod(10);
    kickerConfig.encoder.quadratureAverageDepth(2); // subject to change

    followerFlyConfig.smartCurrentLimit(80);
    followerFlyConfig.idleMode(IdleMode.kCoast);
    followerFlyConfig.follow(mainFly, true);
    followerFlyConfig.disableVoltageCompensation();

    kickerConfig.smartCurrentLimit(80);
    kickerConfig.inverted(false);
    kickerConfig.idleMode(IdleMode.kCoast);
    kickerConfig.disableVoltageCompensation();
    kickerConfig.closedLoop.pid(KICKER_PID[0] / 12.0, KICKER_PID[1] / 12.0, KICKER_PID[2] / 12.0);
    kickerConfig.closedLoop.feedForward.kS(KICKER_FEEDFORWARD.getKs());
    kickerConfig.closedLoop.feedForward.kV(KICKER_FEEDFORWARD.getKv());
    kickerConfig.closedLoop.feedForward.kA(KICKER_FEEDFORWARD.getKa());

    kickerConfig.encoder.positionConversionFactor(KICKER_POS_FACTOR);
    kickerConfig.encoder.velocityConversionFactor(KICKER_VEL_FACTOR);


    mainFly.configure(mainFlyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kicker.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerFly.configure(followerFlyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setFlywheelVoltage(double volts) {
    flywheelController.setSetpoint(volts, ControlType.kVoltage);
  }

  public void setFlywheelVoltage(Voltage volts) {
    setFlywheelVoltage(volts.magnitude());
  }

  public void setKickerVoltage(double volts) {
    kickerController.setSetpoint(volts, ControlType.kVoltage);
  }

  public void setKickerVoltage(Voltage volts) {
    setKickerVoltage(volts.magnitude());
  }

  @AutoLogOutput
  public ControlType getKickerControlType() {
    return kickerController.getControlType();
  }

  @AutoLogOutput
  public ControlType getFlywheelControlType() {
    return flywheelController.getControlType();
  }

  public void setShooterVelocity(double velocity) {
    setFlywheelVelocity(velocity);
    setKickerVelocity(this.getKickerSpeed(velocity));
  }

  private void setFlywheelVelocity(double velocity) {
    flywheelController.setSetpoint(velocity, ControlType.kVelocity);
  }

  private void setKickerVelocity(double velocity) {
    kickerController.setSetpoint(velocity, ControlType.kVelocity);
  }

  @AutoLogOutput
  public double getKickerVelocityRadPerSec() {
    return kickerEncoder.getVelocity();
  }

  @AutoLogOutput
  public double getKickerSetpoint() {
    return kickerController.getSetpoint();
  }

  @AutoLogOutput
  public double getFlywheelVelocityRadPerSec() {
    return flywheelEncoder.getVelocity();
  }

  @AutoLogOutput
  public double getFlywheelSetpoint() {
    return flywheelController.getSetpoint();
  }

  @AutoLogOutput
  public double getFlywheelCurrent() {
    return mainFly.getOutputCurrent();
  }

  @AutoLogOutput
  public double getKickerCurrent() {
    return kicker.getOutputCurrent();
  }

  public Command manualShooter(double volts) {
    return this.run(() -> {
      setKickerVoltage(volts);
      setFlywheelVoltage(-volts);
    }).finallyDo(() -> {
      setKickerVoltage(0);
      setFlywheelVoltage(0);
    });
  }

  public SysIdRoutine getMainFlySysidRoutine() {
    return new SysIdRoutine(new Config(Volts.of(.5).per(Second), Volts.of(7), null, (state) -> {
      Logger.recordOutput("Shooter/mainFlySysidTestState", state.toString());
    }),
        new Mechanism(this::setFlywheelVoltage, null, this));
  }

  public SysIdRoutine getKickerSysidRoutine() {
    return new SysIdRoutine(new Config(Volts.of(.5).per(Second), Volts.of(7), null, (state) -> {
      Logger.recordOutput("Shooter/kickerSysidTestState", state.toString());
    }),
        new Mechanism(this::setKickerVoltage, null, this));
  }

  public Command setObjectiveVelocityCommand(Swerve swerve) {
        return this.run(() -> {
            this.setShooterVelocity(AimingManager.latestShot.flywheelVel());
        });
  }
  public Command setFeedVelocityCommand(Swerve swerve) {
        return this.run(() -> {
            this.setShooterVelocity(AimingManager.latestFeedShot.flywheelVel());
        });
  }
  
  public Command setHubVelocityCommand(Swerve swerve) {
        return this.run(() -> {
            this.setShooterVelocity(AimingManager.latestHubShot.flywheelVel());
        });
  }

  public boolean atSpeed(double flywheelSpeed) {
    boolean mainAtSpeed = MathUtil.isNear(flywheelSpeed, getFlywheelVelocityRadPerSec(), FLYWHEEL_TOLERANCE);
    boolean kickerAtSpeed = MathUtil.isNear(getKickerSpeed(flywheelSpeed), getKickerVelocityRadPerSec(), KICKER_TOLERANCE);
    if (flywheelSpeed > MAX_EFFECTIVE_FLYWHEEL_SPEED) {
      boolean kickerAtMaxSpeed = MathUtil.isNear(MAX_KICKER_SPEED, getKickerVelocityRadPerSec(), KICKER_TOLERANCE);
      return mainAtSpeed && kickerAtMaxSpeed;  // if we are trying to set a speed higher than we can maintain our kicker backspin, then just make sure it's maxxed.
    } else {
      return mainAtSpeed && kickerAtSpeed;
    }
  }

    // see https://gemini.google.com/share/e8d7da86ce5d for explanation, returns motor velocity to get kicker to proportional speed as flywheel; keep in mind flywheel is geared up.
  private double getKickerSpeed(double flywheelSpeed) {
      double kickerSpeed = flywheelSpeed * ShooterConstants.KICKER_SURFACE_SPEED_RATIO * (ShooterConstants.EFFECTIVE_FLYWHEEL_DIAMETER / ShooterConstants.EFFECTIVE_KICKER_DIAMETER);
      return kickerSpeed;
  }

}
