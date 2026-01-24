package frc.robot.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.MiscConstants;

import static edu.wpi.first.math.MathUtil.applyDeadband;

public class Controllers {
     private Controllers() {}

  private static final int MAX_DRIVER_STATION_PORTS = DriverStation.kJoystickPorts; 
  private static final String[] OPERATOR_CONTROLLER_NAMES = {
    "CircuitPython HID",
    "Keyboard 1"
  };
  private static String[] lastControllerNames = new String[MAX_DRIVER_STATION_PORTS];

  public static DriverController driverController;
  public static Macropad operatorController;

  /** Returns true if the connected controllers have changed since last called. */
  public static boolean didControllersChange() {
    boolean hasChanged = false;
    String name;

    for (int i = 0; i < MAX_DRIVER_STATION_PORTS ; i++) {
      name = DriverStation.getJoystickName(i);
      if (!name.equals(lastControllerNames[i])) {
        hasChanged = true;
        lastControllerNames[i] = name;
      }
    }

    return hasChanged;
  }

  public static void updateActiveControllerInstance() {
    boolean foundDriveController = false;
    boolean foundOperatorController = false;
    String joyName;

    driverController = new DriverController(-1);
    operatorController = new Macropad(-1);

    for (int port = 0; port < MAX_DRIVER_STATION_PORTS; port++) {
      if (DriverStation.isJoystickConnected(port)) {
        joyName = DriverStation.getJoystickName(port);

        if (!foundOperatorController && isOperatorControllerName(joyName)) {
          foundOperatorController = true;
          operatorController = new Macropad(port);
        }
        // No filtering for now, just use the first
        else if (!foundDriveController) {
          foundDriveController = true;
          driverController = new DriverController(port);
        }
      }
    }
  }

  private static boolean isOperatorControllerName(String name) {
    for (String controllerName : OPERATOR_CONTROLLER_NAMES) {
      if (name.contains(controllerName)) {
        return true;
      }
    }
    return false;
  }

  public static double deadband(double num) {
    return applyDeadband(num, MiscConstants.JOYSTICK_DEADBAND);
  } 
}

