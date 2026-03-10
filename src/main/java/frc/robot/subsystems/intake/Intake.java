package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.IntakeConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {

  SparkMax intake = new SparkMax(INTAKE_ID, MotorType.kBrushless); // neo 2.0
  RelativeEncoder intakEncoder = intake.getEncoder();
  SparkMaxConfig intakeConfig = new SparkMaxConfig();
  SparkClosedLoopController intakeController = intake.getClosedLoopController();

  boolean isManualMode = false;

  public Intake() {
    configure();
  }

  public void configure() {
    intakeConfig.encoder.quadratureMeasurementPeriod(10);
    intakeConfig.encoder.quadratureAverageDepth(2);

    intakeConfig.smartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);
    intakeConfig.idleMode(IdleMode.kCoast);

    intakeConfig.inverted(true);

    intakeConfig.closedLoop.feedForward.kA(INTAKE_FEEDFORWARD.getKa());
    intakeConfig.closedLoop.feedForward.kV(INTAKE_FEEDFORWARD.getKv());
    intakeConfig.closedLoop.feedForward.kS(INTAKE_FEEDFORWARD.getKs());

    intakeConfig.closedLoop.pid(INTAKE_PID[0] / 12.0, INTAKE_PID[1] / 12.0, INTAKE_PID[2] / 12.0);

    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setIntakeVoltage(Voltage volts) {
    setIntakeVoltage(volts.magnitude());
  }

  public void setIntakeVoltage(double volts) {
    intake.setVoltage(volts);
  }

  public void stopIntake() {
    intake.setVoltage(0);
  }

  @AutoLogOutput
  public double getIntakeVoltage() {
    return intake.get() * intake.getBusVoltage();
  }

  public double getIntakeVelocity() {
    return intakEncoder.getVelocity();
  }

  public void setIntakeVelocity(double velocity) {
    intakeController.setSetpoint(velocity, ControlType.kVelocity);
  }

  public SysIdRoutine getIntakeSysid() {
    return new SysIdRoutine(
        new Config(Volts.of(.5).per(Second), Volts.of(2), null, (state) -> {
          Logger.recordOutput("Intake/intakeSysidTestState", state.toString());
        }),
        new Mechanism(this::setIntakeVoltage, null, this));
  }
}
