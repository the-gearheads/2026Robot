package frc.robot.commands.NTControl;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Hood;

public class HoodNTControl extends Command {
  Hood hood;
  String NTPath = "Hood/manualPosition";
  
  public HoodNTControl(Hood hood) {
    this.hood = hood;
    addRequirements(hood);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber(NTPath, hood.getAngle().getDegrees());
    hood.setAngle(hood.getAngle());
  }

  @Override
  public void execute() {
    hood.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber(NTPath, hood.getAngle().getDegrees())));
  }
}
