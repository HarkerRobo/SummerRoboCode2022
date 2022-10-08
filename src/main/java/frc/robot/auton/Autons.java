package frc.robot.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.hood.ZeroHood;
import frc.robot.commands.intake.IntakeAuton;
import frc.robot.commands.shooter.ShooterAuton;
import frc.robot.util.Constants;

public class Autons {

  public static final SequentialCommandGroup THREE_BALL_AUTO =
      new SequentialCommandGroup(
          new ZeroHood(),
          new WaitCommand(1),
          new ShooterAuton(),
          new SwervePosController(
                  Trajectories.twoBallTop,
                  (point, time) -> getAngleToTarget(point.getTranslation(), Constants.HUB_LOCATION),
                  () -> Rotation2d.fromDegrees(136.97))
              .deadlineWith(new IntakeAuton()),
          new WaitCommand(1),
          new ShooterAuton());

  public static Rotation2d getAngleToTarget(
      Translation2d fieldToRobot, Translation2d fieldToTarget) {
    Translation2d targetToRobot = fieldToTarget.minus(fieldToRobot);
    return new Rotation2d(targetToRobot.getX(), targetToRobot.getY())
        .plus(Rotation2d.fromDegrees(180));
  }
}
