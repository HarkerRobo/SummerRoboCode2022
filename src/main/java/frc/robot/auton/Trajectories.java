package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import frc.robot.subsystems.Drivetrain;
import java.util.ArrayList;
import java.util.List;

public class Trajectories {
  public static TrajectoryConfig defaultConfig =
      new TrajectoryConfig(2, 1.5).setKinematics(Drivetrain.getInstance().getKinematics());

  public static Trajectory twoBallTop =
      generateTrajectory(
          List.of(
              new Pose2d(6.07, 5.22, Rotation2d.fromDegrees(136.97)), // 6.22, 5.18
              new Pose2d(5.29, 5.89, Rotation2d.fromDegrees(136.97))),
          2.0,
          1.0,
          0.0,
          0.0);

  public static Trajectory generateTrajectory(
      List<Pose2d> points,
      double maxVel,
      double maxAccel,
      double startVel,
      double endVel,
      TrajectoryConstraint... constraints) {
    TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
    for (TrajectoryConstraint c : constraints) {
      config.addConstraint(c);
    }
    config.setStartVelocity(startVel);
    config.setEndVelocity(endVel);
    List<Translation2d> interiorPoints = new ArrayList<Translation2d>();
    for (int i = 1; i < points.size() - 1; i++) {
      interiorPoints.add(points.get(i).getTranslation());
    }
    return TrajectoryGenerator.generateTrajectory(
        points.get(0), interiorPoints, points.get(points.size() - 1), config);
  }
}
