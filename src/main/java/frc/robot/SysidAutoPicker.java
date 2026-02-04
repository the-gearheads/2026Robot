package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class SysidAutoPicker {
  private SendableChooser<Command> chooser = new SendableChooser<>();

  public SysidAutoPicker() {
    SmartDashboard.putData("SysidRoutines", chooser);
  }

  public void addSysidCommand(Command command, String name) {
    chooser.addOption(name, command);
  }

  /**
   *  Add all relevant sysidRoutines within constraints to the chooser
   * @param name Name to append routine types to in chooser
   * @param routine Routine to use
   * @param forwardLimit Should return true if subsytem is past a safe position in the forward direction
   * @param reverseLimit Should return true if subsystem is past a safe position in the reverse direction
   */
  public void addSysidRoutines(String name, SysIdRoutine routine, BooleanSupplier forwardLimit, BooleanSupplier reverseLimit) {
    chooser.addOption(name + " Quasi ->", routine.quasistatic(Direction.kForward).until(forwardLimit));
    chooser.addOption(name + " Quasi <-", routine.quasistatic(Direction.kReverse).until(reverseLimit));
    chooser.addOption(name + " Dynamic ->", routine.dynamic(Direction.kForward).until(forwardLimit));
    chooser.addOption(name + " Dynamic <-", routine.dynamic(Direction.kReverse).until(reverseLimit));
  }

  public void addSysidRoutines(String name, SysIdRoutine routine) {
    addSysidRoutines(name, routine, () -> false, () -> false);
  }

  public Command get() {
    return chooser.getSelected();
  }
}
