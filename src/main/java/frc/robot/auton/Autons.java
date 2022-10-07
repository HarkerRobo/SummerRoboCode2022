package frc.robot.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.hood.ZeroHood;
import frc.robot.commands.intake.IntakeAuton;
import frc.robot.commands.shooter.ShooterAuton;

public class Autons {

  public static final SequentialCommandGroup THREE_BALL_AUTO =
      new SequentialCommandGroup(
          new ZeroHood(),
          new ShooterAuton(),
          new SwervePosController(
                  Trajectories.twoBallTop,
                  (point, time) -> Rotation2d.fromDegrees(136.97),
                  () -> Rotation2d.fromDegrees(136.97))
              .deadlineWith(new IntakeAuton()),
          new ShooterAuton());
}
