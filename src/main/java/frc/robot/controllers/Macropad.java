package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class Macropad {
    Joystick joy;
  public Macropad(int id) {
    if(id == -1) {
      this.joy = null;
      return;
    }
    joy = new Joystick(id);
  }
  

  private boolean isNull() {
    return joy == null;
  }

  @SuppressWarnings("unused")
  private Trigger emptyTrigger() {
    return new Trigger(() -> false);
  }

  public double getRotencAxis() {
    if(isNull()) return 0;
    return joy.getRawAxis(0);
  }

  public Trigger getBtn11() {
    if(isNull()) return emptyTrigger();
    return new Trigger(()->joy.getRawButton(1));
  }

  public Trigger getBtn12() {
    if(isNull()) return emptyTrigger();
    return new Trigger(()->joy.getRawButton(2));
  }

  public Trigger getBtn13() {
    if(isNull()) return emptyTrigger();
    return new Trigger(()->joy.getRawButton(3));
  }

  public Trigger getBtn21() {
    if(isNull()) return emptyTrigger();
    return new Trigger(()->joy.getRawButton(4));
  }

  public Trigger getBtn22() {
    if(isNull()) return emptyTrigger();
    return new Trigger(()->joy.getRawButton(5));
  }

  public Trigger getBtn23() {
    if(isNull()) return emptyTrigger();
    return new Trigger(()->joy.getRawButton(6));
  }

  public Trigger getBtn31() {
    if(isNull()) return emptyTrigger();
    return new Trigger(()->joy.getRawButton(7));
  }

  public Trigger getBtn32() {
    if(isNull()) return emptyTrigger();
    return new Trigger(()->joy.getRawButton(8));
  }

  public Trigger getBtn33() {
    if(isNull()) return emptyTrigger();
    return new Trigger(()->joy.getRawButton(9));
  }

  public Trigger getBtn41() {
    if(isNull()) return emptyTrigger();
    return new Trigger(()->joy.getRawButton(10));
  }

  public Trigger getBtn42() {
    if(isNull()) return emptyTrigger();
    return new Trigger(()->joy.getRawButton(11));
  }

  public Trigger getBtn43() {
    if(isNull()) return emptyTrigger();
    return new Trigger(()->joy.getRawButton(12));
  }

  public Trigger getRotencBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(()->joy.getRawButton(13));
  }
}
