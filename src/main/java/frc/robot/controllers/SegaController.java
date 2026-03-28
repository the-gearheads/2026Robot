package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SegaController implements OperatorController{
    Joystick joy;

    public SegaController(int id) {
        if (id == -1) {
            this.joy = null;
            return;
        }
        joy = new Joystick(id);
    }

    private boolean isNull() {
        return joy == null;
    }

    private Trigger emptyTrigger() {
        return new Trigger(() -> false);
    }

    public Trigger getAButton() {
        if(isNull()) return emptyTrigger();
        return new Trigger(()-> joy.getRawButton(2));
    }

    
    public Trigger getBButton() {
        if(isNull()) return emptyTrigger();
        return new Trigger(()-> joy.getRawButton(1));
    }

    
    public Trigger getYButton() {
        if(isNull()) return emptyTrigger();
        return new Trigger(()-> joy.getRawButton(3));
    }

    
    public Trigger getXButton() {
        if(isNull()) return emptyTrigger();
        return new Trigger(()-> joy.getRawButton(4));
    }

    public Trigger getLeftBumper() {
        if(isNull()) return emptyTrigger();
        return new Trigger(()-> joy.getRawButton(7));
    }

    public Trigger getRightBumper() {
        if(isNull()) return emptyTrigger();
        return new Trigger(()-> joy.getRawButton(8));
    }

    public Trigger getCButton() {
        if (isNull()) return emptyTrigger();
        return new Trigger(()-> joy.getRawButton(5));
    }

}
