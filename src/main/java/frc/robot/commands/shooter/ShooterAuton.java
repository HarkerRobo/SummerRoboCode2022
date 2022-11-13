package frc.robot.commands.shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShooterAuton extends CommandBase {

  public ShooterAuton() {
    addRequirements(Shooter.getInstance(), Drivetrain.getInstance());
  }

  public void initialize() {
    Shooter.getInstance().setState(Shooter.State.REVVING);
  }

  public void execute() {
    double omega = Drivetrain.getInstance().alignWithHub();
    omega *= Drivetrain.getInstance().alignWithHub();
    omega *= SwerveManual.SPEED_MULTIPLIER;
    ChassisSpeeds chassis = new ChassisSpeeds(0, 0, omega);
    Drivetrain.getInstance().setAngleAndDrive(chassis);
    double speed = Shooter.getInstance().calculateShooterSpeed();
    Shooter.getInstance().set(speed);
    if (Shooter.getInstance().atSpeed(speed) && Hood.getInstance().atHoodAngle())
      Shooter.getInstance().setState(Shooter.State.SHOOTING);
    else Shooter.getInstance().setState(Shooter.State.REVVING);
  }

  public boolean isFinished() {
    return Indexer.getInstance().isEmpty();
  }

  public void end(boolean interruped) {
    Shooter.getInstance().setState(Shooter.State.IDLE);
    Shooter.getInstance().turnOffMotors();
    Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds(0, 0, SwerveManual.MIN_OUTPUT));
  }
}
