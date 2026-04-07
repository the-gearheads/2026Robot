package frc.robot.controllers;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SegaController implements OperatorController{
    Joystick joy;

    public SegaController(int id) {
        joy = new Joystick(id);
    }

    public Trigger getAButton() {
        return new Trigger(()-> joy.getRawButton(2));
    }

    
    public Trigger getBButton() {
        return new Trigger(()-> joy.getRawButton(3));
    }

    
    public Trigger getYButton() {
        return new Trigger(()-> joy.getRawButton(4));
    }

    
    public Trigger getXButton() {
        return new Trigger(()-> joy.getRawButton(1));
    }

    public Trigger getLeftBumper() {
        return new Trigger(()-> joy.getRawButton(7));
    }

    public Trigger getRightBumper() {
        return new Trigger(()-> joy.getRawButton(8));
    }

    public Trigger getCButton() {;
        return new Trigger(()-> joy.getRawButton(5));
    }

}
