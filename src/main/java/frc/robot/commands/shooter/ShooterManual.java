package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.State;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.PhotonVisionLimelight;
import harkerrobolib.commands.IndefiniteCommand;

public class ShooterManual extends IndefiniteCommand {
  private InterpolatingTreeMap shooterVals;
  private double smartdashboard;

  public ShooterManual() {
    addRequirements(Shooter.getInstance());
    shooterVals = new InterpolatingTreeMap();
    shooterVals.put(1.15, 10.5);
    shooterVals.put(2.8, 10.3);
    shooterVals.put(3.2, 11.0);
    shooterVals.put(3.4, 11.5);
    shooterVals.put(3.7, 11.7);
    shooterVals.put(3.9, 11.8);
    shooterVals.put(4.19, 12.0);
    shooterVals.put(4.57, 12.1);
    shooterVals.put(4.85, 12.6);
    shooterVals.put(5.2, 12.7);
    shooterVals.put(5.4, 13.1);
    shooterVals.put(5.78, 13.5);
    shooterVals.put(6.26, 14.0);
  }

  public void execute() {
    double possibleShooterSpeed = calculateShooterSpeed();
    SmartDashboard.putNumber("speed", smartdashboard);
    updateShooterState(possibleShooterSpeed);
    if (Shooter.getInstance().getState() != State.IDLE) {
      Shooter.getInstance().set(possibleShooterSpeed);
    } else Shooter.getInstance().turnOffMotors();
  }

  private void updateShooterState(double nextSpeed) {
    SmartDashboard.putBoolean("Y", OI.getInstance().getDriverGamepad().getButtonY().get());
    if (OI.getInstance().getDriverGamepad().getButtonY().get()) {
      switch (Shooter.getInstance().getState()) {
        case IDLE:
          Shooter.getInstance().setState(State.REVVING);
          break;
        case REVVING:
          if (Shooter.getInstance().atSpeed(nextSpeed))// && Drivetrain.getInstance().isAligned())
            Shooter.getInstance().setState(State.SHOOTING);
          break;
        case SHOOTING:
          if (!(Shooter.getInstance().atSpeed(nextSpeed)))// && Drivetrain.getInstance().isAligned()))
            Shooter.getInstance().setState(State.REVVING);
      }
    } else {
      Shooter.getInstance().setState(State.IDLE);
    }
  }

  private double calculateShooterSpeed() {
    // smartdashboard = SmartDashboard.getNumber("speed", 0.0);
    // return smartdashboard;
    return shooterVals.get(PhotonVisionLimelight.getDistance());
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
