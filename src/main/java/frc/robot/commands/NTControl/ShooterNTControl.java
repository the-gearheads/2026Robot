package frc.robot.commands.NTControl;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterNTControl extends Command {
  Shooter shooter;
  String NTPath = "Shooter/manSpeed";
  
  public ShooterNTControl(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber(NTPath, Units.radiansPerSecondToRotationsPerMinute(shooter.getFlywheelVelocityRadPerSec()));
    shooter.setShooterVelocity(0);
  }

  @Override
  public void execute() {
    shooter.setShooterVelocity(Units.rotationsPerMinuteToRadiansPerSecond(SmartDashboard.getNumber(NTPath, 0)));
  }
}
