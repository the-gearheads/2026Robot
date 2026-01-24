package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverController {
  XboxController controller;

  public DriverController(int id) {
    if(id == -1) {
      this.controller = null;
      return;
    }
    this.controller = new XboxController(id);
  }

  private boolean isNull() {
    return controller == null;
  }

  private Trigger emptyTrigger() {
    return new Trigger(() -> false);
  }

  public double getTranslateXAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(-controller.getLeftY());
  }

  public double getTranslateYAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(-controller.getLeftX());
  }

  public double getRotateAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(-controller.getRightX());
  }

  public double getLeftTriggerAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(controller.getLeftTriggerAxis());
  }
  
  public double getRightTriggerAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(controller.getRightTriggerAxis());
  }

  public Trigger getXBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getXButton());  }

  public Trigger getABtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getAButton());  }

  public Trigger getYBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getYButton());
  }

  public Trigger getBBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getBButton());
  }

  public Trigger getLeftBumper() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getLeftBumperButton());
  }

  public Trigger getRightBumper() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getRightBumperButton());
  }
  
  public Trigger getPovLeft() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getPOV() == 270);
  }
  
  public Trigger getPovRight() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getPOV() == 90);
  }
  
  public Trigger getPovUp() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getPOV() == 0);
  }
  
  public Trigger getPovDown() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getPOV() == 180);
  }
  
  public Trigger getLeftPaddle() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getLeftStickButton());
  }

  public Trigger getRightPaddle() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getRightStickButton());
  }

  public Trigger getBackButton() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getBackButton());
  }

  public Trigger getStartButton() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getStartButton());
  }

  public Trigger getLeftTriggerBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getLeftTriggerAxis() > 0.05);
  }

  public Trigger getRightTriggerBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getRightTriggerAxis() > 0.05);
  }

  public void setRumble(double rumble) {
    if(isNull()) return;
    controller.setRumble(RumbleType.kBothRumble, rumble);
  }

  public Trigger getRawButton(int button) {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getRawButton(button));

  }
  public Command getRumbleCommand(double rumble, double seconds) {
    return Commands.runEnd(()->setRumble(rumble), ()->setRumble(0)).withTimeout(seconds);
  }

  public Command getRumbleCommand(double rumble, double seconds, int pulses) {
    final int pulseCounter[] = {0};
    return Commands.runEnd(
      ()->setRumble(rumble),
      ()->{
        setRumble(0);
        pulseCounter[0]++;
      })
      .withTimeout(seconds)
      .andThen(new WaitCommand(seconds*1.2))
      .repeatedly()
      .until(() -> pulseCounter[0]==pulses);
  }
}
