package frc.robot.commands.auton;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.loop.PositionControlLoop;
import frc.robot.util.loop.PositionControlLoop.PositionControlLoopBuilder;

public class SwerveControllerCommand extends CommandBase {

  private static final double TRANSLATION_MAX_VOLTAGE = 1; // TODO
  private static final double THETA_MAX_VOLTAGE = 1; // TODO

  private static final double TRANSLATION_POS_TOLERANCE = 0.01; // TODO
  private static final double ROTATION_POS_TOLERANCE = 0.01; // TODO
  private static final double TRANSLATION_VEL_TOLERANCE = 0.01; // TODO
  private static final double ROTATION_VEL_TOLERANCE = 0.01; // TODO

  private static final double TRANS_MAX_POS_ERROR = 5; // TODO
  private static final double TRANS_MAX_VEL_ERROR = 5; // TODO
  private static final double TRANS_MODEL_POS_STDEV = 0.5; // TODO
  private static final double TRANS_MODEL_VEL_STDEV = 0.5; // TODO
  private static final double TRANS_MEAS_STDEV = 0.035; // TODO

  private static final double THETA_MAX_POS_ERROR = 5; // TODO
  private static final double THETA_MAX_VEL_ERROR = 5; // TODO
  private static final double THETA_MODEL_POS_STDEV = 0.5; // TODO
  private static final double THETA_MODEL_VEL_STDEV = 0.5; // TODO
  private static final double THETA_MEAS_STDEV = 0.035; // TODO

  public static final Matrix<N2, N2> A =
      Matrix.mat(Nat.N2(), Nat.N2()).fill(0.0, 1.0, 0.0, -1 / RobotMap.ROBOT_LOOP);
  public static final Matrix<N2, N1> B = VecBuilder.fill(0.0, 1.0 / RobotMap.ROBOT_LOOP);
  public static final Matrix<N1, N2> C = Matrix.mat(Nat.N1(), Nat.N2()).fill(1.0, 0.0);
  public static final Matrix<N1, N1> D = VecBuilder.fill(0.0);

  private static final PositionControlLoop X_LOOP =
      new PositionControlLoopBuilder()
          .stateMatrices(A, B, C, D)
          .standardDeviations(TRANS_MODEL_POS_STDEV, TRANS_MODEL_VEL_STDEV, TRANS_MEAS_STDEV)
          .maxError(TRANS_MAX_POS_ERROR, TRANS_MAX_VEL_ERROR)
          .maxControlEffort(TRANSLATION_MAX_VOLTAGE)
          .buildPositionControlLoop();
  private static final PositionControlLoop Y_LOOP =
      new PositionControlLoopBuilder()
          .stateMatrices(A, B, C, D)
          .standardDeviations(TRANS_MODEL_POS_STDEV, TRANS_MODEL_VEL_STDEV, TRANS_MEAS_STDEV)
          .maxError(TRANS_MAX_POS_ERROR, TRANS_MAX_VEL_ERROR)
          .maxControlEffort(TRANSLATION_MAX_VOLTAGE)
          .buildPositionControlLoop();
  private static final PositionControlLoop THETA_LOOP =
      new PositionControlLoopBuilder()
          .stateMatrices(A, B, C, D)
          .standardDeviations(THETA_MODEL_POS_STDEV, THETA_MODEL_VEL_STDEV, THETA_MEAS_STDEV)
          .maxError(THETA_MAX_POS_ERROR, THETA_MAX_VEL_ERROR)
          .maxControlEffort(THETA_MAX_VOLTAGE)
          .buildPositionControlLoop();

  public SwerveControllerCommand(Pose2d endpoint) {
    addRequirements(Drivetrain.getInstance());
    X_LOOP.setNextSetpoint(endpoint.getTranslation().getX(), 0.0);
    Y_LOOP.setNextSetpoint(endpoint.getTranslation().getY(), 0.0);
    THETA_LOOP.setNextSetpoint(endpoint.getRotation().getRadians(), 0.0);
  }

  public void initialize() {
    Pose2d initPosition = Drivetrain.getInstance().getPoseEstimator().getEstimatedPosition();
    ChassisSpeeds initVelocity = Drivetrain.getInstance().getChassisSpeeds();
    X_LOOP.reset(initPosition.getX(), initVelocity.vxMetersPerSecond);
    Y_LOOP.reset(initPosition.getY(), initVelocity.vyMetersPerSecond);
    THETA_LOOP.reset(initPosition.getRotation().getRadians(), initVelocity.omegaRadiansPerSecond);
  }

  public void execute() {
    Pose2d currentPose = Drivetrain.getInstance().getPoseEstimator().getEstimatedPosition();
    double vx = X_LOOP.correctAndPredict(currentPose.getX());
    double vy = Y_LOOP.correctAndPredict(currentPose.getY());
    double omega = THETA_LOOP.correctAndPredict(currentPose.getRotation().getRadians());
    Drivetrain.getInstance()
        .setAngleAndDrive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                vx, vy, omega, Rotation2d.fromDegrees(Drivetrain.getInstance().getRobotHeading())));
  }

  public boolean isFinished() {
    double xPosError = Math.abs(X_LOOP.getFilteredPosition() - X_LOOP.getSetpointPosition());
    double yPosError = Math.abs(X_LOOP.getFilteredPosition() - X_LOOP.getSetpointPosition());
    double thetaPosError = Math.abs(X_LOOP.getFilteredPosition() - X_LOOP.getSetpointPosition());
    double xVelError = Math.abs(X_LOOP.getFilteredVelocity() - X_LOOP.getSetpointVelocity());
    double yVelError = Math.abs(X_LOOP.getFilteredVelocity() - X_LOOP.getSetpointVelocity());
    double thetaVelError = Math.abs(X_LOOP.getFilteredVelocity() - X_LOOP.getSetpointVelocity());
    return xPosError < TRANSLATION_POS_TOLERANCE
        && yPosError < TRANSLATION_POS_TOLERANCE
        && thetaPosError < ROTATION_POS_TOLERANCE
        && xVelError < TRANSLATION_VEL_TOLERANCE
        && yVelError < TRANSLATION_VEL_TOLERANCE
        && thetaVelError < ROTATION_VEL_TOLERANCE;
  }
}
