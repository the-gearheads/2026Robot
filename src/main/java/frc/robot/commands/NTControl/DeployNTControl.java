package frc.robot.commands.NTControl;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Deploy;

public class DeployNTControl extends Command {
  Deploy deploy;
  String NTPath = "deploy/manualPosition";
  
  public DeployNTControl(Deploy deploy) {
    this.deploy = deploy;
    addRequirements(deploy);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber(NTPath, deploy.getAngle().getDegrees());
    deploy.setAngle(deploy.getAngle());
  }

  @Override
  public void execute() {
    deploy.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber(NTPath, deploy.getAngle().getDegrees())));
  }
}
