package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorController {
    
    public default Trigger getAButton(){
        return new Trigger(() ->false);
    }
    
     public default Trigger getBButton(){
        return new Trigger(() ->false);
    }

     public default Trigger getXButton(){
        return new Trigger(() ->false);
    }

     public default Trigger getYButton(){
        return new Trigger(() ->false);
    }

      public default Trigger getCButton(){
        return new Trigger(() ->false);
    }

    public   default Trigger getLeftBumper() {
        return new Trigger(() ->false);
    }

    public  default Trigger getRightBumper() {
        return new Trigger(() ->false);
    }

    public default Trigger getStartButton() {;
        return new Trigger(()-> false);
    }
    public default Trigger getPOVUp() {
        return new Trigger(()-> false);
    }
    public default Trigger getPOVDown(){
        return new Trigger(() -> false);
    }

}
