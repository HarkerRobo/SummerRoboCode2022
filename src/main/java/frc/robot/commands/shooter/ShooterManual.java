package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.State;
import frc.robot.util.InterpolatingTreeMap;
import harkerrobolib.commands.IndefiniteCommand;

public class ShooterManual extends IndefiniteCommand {
  private InterpolatingTreeMap shooterVals;

  public ShooterManual() {
    addRequirements(Shooter.getInstance());
    shooterVals = new InterpolatingTreeMap();
  }

  public void execute() {
    double possibleShooterSpeed = calculateShooterSpeed();
    updateShooterState(possibleShooterSpeed);
    if (Shooter.getInstance().getState() != State.IDLE) {
      Shooter.getInstance().set(possibleShooterSpeed);
    } else Shooter.getInstance().turnOffMotors();
    SmartDashboard.putNumber("shooter counter", Robot.counter);
  }

  private void updateShooterState(double nextSpeed) {
    SmartDashboard.putBoolean("Y", OI.getInstance().getDriverGamepad().getButtonY().get());
    if (OI.getInstance().getDriverGamepad().getButtonY().get()) {
      switch (Shooter.getInstance().getState()) {
        case IDLE:
          Shooter.getInstance().setState(State.REVVING);
          break;
        case REVVING:
          // if (Shooter.getInstance().atSpeed(nextSpeed) && Drivetrain.getInstance().isAligned())
          //   Shooter.getInstance().setState(State.SHOOTING);
          break;
        case SHOOTING:
          if (!(Shooter.getInstance().atSpeed(nextSpeed) && Drivetrain.getInstance().isAligned()))
            Shooter.getInstance().setState(State.REVVING);
      }
    } else {
      Shooter.getInstance().setState(State.IDLE);
    }
    SmartDashboard.putNumber("shooter counter", Robot.counter);
  }

  private double calculateShooterSpeed() {
    return 15;
    // return shooterVals.get(
    //     Drivetrain.getInstance()
    //         .getPoseEstimator()
    //         .getEstimatedPosition()
    //         .getTranslation()
    //         .getDistance(FieldConstants.HUB_LOCATION));
  }

  public void end(boolean interrupted) {
    Shooter.getInstance().turnOffMotors();
  }
}
