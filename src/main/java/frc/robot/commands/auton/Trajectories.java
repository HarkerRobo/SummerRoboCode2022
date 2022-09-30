package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;
import java.util.ArrayList;

public class Trajectories {
  public static TrajectoryConfig defaultConfig =
      new TrajectoryConfig(2, 1.5).setKinematics(Drivetrain.getInstance().getKinematics());

  public static final Pose2d[] hAG = {
    new Pose2d(new Translation2d(6.8, 5.63), Rotation2d.fromDegrees(-65.42)),
    new Pose2d(new Translation2d(6.28, 6.79), Rotation2d.fromDegrees(-65.42)),
    new Pose2d(new Translation2d(1.67, 6.56), Rotation2d.fromDegrees(-90.0)),
    new Pose2d(new Translation2d(1.85, 4.41), Rotation2d.fromDegrees(180.0)),
    new Pose2d(new Translation2d(4.03, 3.29), Rotation2d.fromDegrees(135.59)),
    new Pose2d(new Translation2d(5.12, 3.64), Rotation2d.fromDegrees(-167.83))
  };
  public static final Pose2d[] hFFB = {
    new Pose2d(new Translation2d(6.18, 3.99), Rotation2d.fromDegrees(26.92)),
    new Pose2d(new Translation2d(5.02, 3.47), Rotation2d.fromDegrees(20.20)),
    new Pose2d(new Translation2d(1.82, 4.44), Rotation2d.fromDegrees(0.0)),
    new Pose2d(new Translation2d(2.22, 6.49), Rotation2d.fromDegrees(-90.0)),
    new Pose2d(new Translation2d(5.54, 7.4), Rotation2d.fromDegrees(166.27)),
    new Pose2d(new Translation2d(6.41, 5.82), Rotation2d.fromDegrees(128.29))
  };

  public static final Pose2d[] tBA = {
    new Pose2d(new Translation2d(7.436414, 2.015084), Rotation2d.fromDegrees(90)),
    new Pose2d(new Translation2d(7.45043, 0.821275), Rotation2d.fromDegrees(90))
  };

  public static final Trajectory hoarderAgainstGremlin = generateTrajectory(hAG, defaultConfig);
  public static final Trajectory hoarderForFiveBall = generateTrajectory(hFFB, defaultConfig);
  public static final Trajectory twoBallAuto = generateTrajectory(tBA, defaultConfig);

  private static Trajectory generateTrajectory(Pose2d[] poses, TrajectoryConfig config) {
    ArrayList<Translation2d> interior = new ArrayList<>();
    for (int i = 1; i < poses.length - 1; i++) {
      interior.add(poses[i].getTranslation());
    }
    return TrajectoryGenerator.generateTrajectory(
        poses[0], interior, poses[poses.length - 1], config);
  }
}
