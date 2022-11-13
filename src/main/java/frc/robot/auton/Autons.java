package frc.robot.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.hood.HoodManual;
import frc.robot.commands.hood.ZeroHood;
import frc.robot.commands.indexer.IndexerManual;
import frc.robot.commands.shooter.ShooterAuton;
import frc.robot.util.Constants;

public class Autons {

  public static final ParallelCommandGroup THREE_BALL_AUTO =
      new ParallelCommandGroup(
          new SequentialCommandGroup(
              new ShooterAuton(),
              new SwervePosController(
                  Trajectories.twoBallTop,
                  (point, time) ->
                      getAngleToTarget(point.getTranslation(), Constants.HUB_LOCATION)
                          .minus(Rotation2d.fromDegrees(20)),
                  () -> Rotation2d.fromDegrees(136.97)),
              // .deadlineWith(new IntakeAuton()),
              new WaitCommand(1),
              new ShooterAuton()),
          new SequentialCommandGroup(new ZeroHood(), new HoodManual()),
          new IndexerManual());

  public static final ParallelCommandGroup TEST_AUTON =
      new ParallelCommandGroup(
          new SequentialCommandGroup(
              new SwervePosController(
                  Trajectories.testAuto,
                  (point, time) -> Rotation2d.fromDegrees(0),
                  () -> Rotation2d.fromDegrees(-90)),
             new SwervePosController(
                 Trajectories.testAuto1,
                 (point, time) -> Rotation2d.fromDegrees(45),
                 () -> Rotation2d.fromDegrees(0))
             )
           );

  public static Rotation2d getAngleToTarget(
      Translation2d fieldToRobot, Translation2d fieldToTarget) {
    Translation2d targetToRobot = fieldToTarget.minus(fieldToRobot);
    return new Rotation2d(targetToRobot.getX(), targetToRobot.getY())
        .plus(Rotation2d.fromDegrees(180));
  }
}
