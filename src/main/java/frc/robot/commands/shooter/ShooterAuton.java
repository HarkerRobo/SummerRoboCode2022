package frc.robot.commands.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShooterAuton extends CommandBase {
  private Debouncer debouncerIndexer;

  public ShooterAuton() {
    addRequirements(Shooter.getInstance(), Drivetrain.getInstance());
  }

  public void initialize() {
    Shooter.getInstance().setState(Shooter.State.REVVING);
  }

  public void execute() {
    double speed = Shooter.getInstance().calculateShooterSpeed();
    Shooter.getInstance().set(speed);
    ChassisSpeeds chassis = new ChassisSpeeds(0, 0, Drivetrain.getInstance().alignWithHub());
    Drivetrain.getInstance().setAngleAndDrive(chassis);
    if (Shooter.getInstance().atSpeed(speed))
      Shooter.getInstance().setState(Shooter.State.SHOOTING);
    else Shooter.getInstance().setState(Shooter.State.REVVING);
  }

  public boolean isFinished() {
    return Indexer.getInstance().isEmpty();
  }

  public void end(boolean interruped) {
    Shooter.getInstance().setState(Shooter.State.IDLE);
    Shooter.getInstance().turnOffMotors();
  }
}
