package frc.robot.commands.NTControl;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class DeployNTControl extends Command {
  Intake intake;
  String NTPath = "deploy/manualPosition";
  
  public DeployNTControl(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber(NTPath, intake.getAngle().getDegrees());
    intake.setAngle(intake.getAngle());
  }

  @Override
  public void execute() {
    intake.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber(NTPath, intake.getAngle().getDegrees())));
  }
}
