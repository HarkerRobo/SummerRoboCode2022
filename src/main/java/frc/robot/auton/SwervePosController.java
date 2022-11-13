package frc.robot.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import java.util.function.BiFunction;
import java.util.function.Supplier;

public class SwervePosController extends CommandBase {
  public static final double X_KP = 4.5;
  public static final double X_KI = 0;
  public static final double X_KD = 0;

  public static final double Y_KP = 3;
  public static final double Y_KI = 0;
  public static final double Y_KD = 0;

  public static final double THETA_KP = 3.7;
  public static final double THETA_KI = 0.00;
  public static final double THETA_KD = 0.0;

  public static double MAX_ANGLE_VELOCITY = Math.PI;
  public static double MAX_ANGLE_ACCELERATION = Math.PI / 2;

  private static PIDController xController = new PIDController(X_KP, X_KI, X_KD);
  private static PIDController yController = new PIDController(Y_KP, Y_KI, Y_KD);
  private static ProfiledPIDController thetaController =
      new ProfiledPIDController(
          THETA_KP,
          THETA_KI,
          THETA_KD,
          new Constraints(MAX_ANGLE_VELOCITY, MAX_ANGLE_ACCELERATION));

  private final Trajectory trajectory;
  private final BiFunction<Pose2d, Double, Rotation2d> refHeading;
  private final Supplier<Rotation2d> startHeading;
  private final Timer timer = new Timer();

  public SwervePosController(
      Trajectory trajectory,
      BiFunction<Pose2d, Double, Rotation2d> refHeading,
      Supplier<Rotation2d> startHeading) {
    this.trajectory = trajectory;
    this.refHeading = refHeading;
    this.startHeading = startHeading;
    thetaController.enableContinuousInput(0, 2 * Math.PI);
    addRequirements(Drivetrain.getInstance());
  }

  public SwervePosController(
      Trajectory trajectory, BiFunction<Pose2d, Double, Rotation2d> refHeading) {
    this(trajectory, refHeading, null);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    Trajectory.State goal = trajectory.sample(timer.get());
    double xFF = goal.velocityMetersPerSecond * goal.poseMeters.getRotation().getCos();
    double yFF = goal.velocityMetersPerSecond * goal.poseMeters.getRotation().getSin();
    Rotation2d angleRef =
        refHeading.apply(Drivetrain.getInstance().getPoseEstimatorPose2d(), timer.get());

    Pose2d currentPose = Drivetrain.getInstance().getPoseEstimatorPose2d();
    double clampAdd =
        1
            + Math.abs(angleRef.getRadians() - currentPose.getRotation().getRadians())
                * (2 / Math.PI);
    double thetaFF =
        MathUtil.clamp(
            thetaController.calculate(
                currentPose.getRotation().getRadians(), angleRef.getRadians()),
            -clampAdd,
            clampAdd);
    // poseError = poseRef.relativeTo(currentPose);
    Rotation2d rotationError = angleRef.minus(currentPose.getRotation());

    // Calculate feedback velocities (based on position error).
    double xFeedback = xController.calculate(currentPose.getX(), goal.poseMeters.getX());
    double yFeedback = yController.calculate(currentPose.getY(), goal.poseMeters.getY());

    // Return next output.
    ChassisSpeeds adjustedSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
    Drivetrain.getInstance().setAngleAndDrive(adjustedSpeeds);
    // Pose2d poseError = autonomusController.m_poseError;
    // Rotation2d rotError = autonomusController.m_rotationError;

    // SmartDashboard.putNumber("Traj-X-Error", Units.metersToInches(poseError.getX()));
    // SmartDashboard.putNumber("Traj-Y-Error", Units.metersToInches(poseError.getY()));
    // SmartDashboard.putNumber("Traj-Theta-Error", rotationError.getDegrees());
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= trajectory.getTotalTimeSeconds();
  }

  @Override
  public void end(boolean interrupted) {
    Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds(0, 0, 0));
  }
}
