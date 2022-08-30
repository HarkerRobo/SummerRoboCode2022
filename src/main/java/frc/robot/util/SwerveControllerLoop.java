package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

public class SwerveControllerLoop {
  private static double[] positionGains = new double[3];
  private static double[] velocityGains = new double[3];

  static {
    Matrix<N2, N2> A = Matrix.mat(Nat.N2(), Nat.N2()).fill(0.0, 1.0, 0.0, -1 / RobotMap.ROBOT_LOOP);
    Matrix<N2, N1> B = VecBuilder.fill(0.0, 1.0 / RobotMap.ROBOT_LOOP);
    final double MAX_TRANSLATION_POS_ERROR = 0.1;  // TODO: Tune
    final double MAX_ROTATION_POS_ERROR = 0.1; // TODO: Tune
    final double MAX_TRANSLATION_VEL_ERROR = 0.1; // TODO: Tune
    final double MAX_ROTATION_VEL_ERROR = 0.1; // TODO: Tune
    double[] translationGains =
        new LinearQuadraticRegulator<N2, N1, N1>(
                A,
                B,
                VecBuilder.fill(MAX_TRANSLATION_POS_ERROR, MAX_TRANSLATION_VEL_ERROR),
                VecBuilder.fill(Drivetrain.MAX_TRANSLATION_VEL),
                RobotMap.ROBOT_LOOP)
            .getK()
            .getData();
    double[] rotationGains =
        new LinearQuadraticRegulator<N2, N1, N1>(
                A,
                B,
                VecBuilder.fill(MAX_ROTATION_POS_ERROR, MAX_ROTATION_VEL_ERROR),
                VecBuilder.fill(Drivetrain.MAX_ROTATION_VEL),
                RobotMap.ROBOT_LOOP)
            .getK()
            .getData();
    positionGains[0] = positionGains[1] = translationGains[0];
    positionGains[2] = rotationGains[0];
    velocityGains[0] = velocityGains[1] = translationGains[1];
    velocityGains[2] = rotationGains[1];
  }

  public static double getRotationOnlyOutput(double curRotVelocity, double headingSetpoint, double headingVelSetpoint) {
    double curHeading = Drivetrain.getInstance().getPoseEstimator().getEstimatedPosition().getRotation().getDegrees();
    while(Math.abs(headingSetpoint - curHeading) > 360.0) {
      headingSetpoint += Math.signum(headingSetpoint - curHeading) * 360.0;
    }
    return positionGains[2] * (headingSetpoint - curHeading) + velocityGains[2] * (headingVelSetpoint - curRotVelocity);
  }

  public static double getRotationOnlyOutput(double curRotVelocity, double headingSetpoint) {
    return getRotationOnlyOutput(curRotVelocity, headingSetpoint, 0.0);
  }
}
