// package frc.robot.util;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;

// public class SwervePoseEstimator extends SwerveDrivePoseEstimator{
//     private Pose2d prevPose;
//     private double prevTime;

//     public SwervePoseEstimator(Rotation2d gyroAngle, Pose2d initialPoseMeters, SwerveDriveKinematics kinematics,
//             Matrix<N3, N1> stateStdDevs, Matrix<N1, N1> localMeasurementStdDevs,
//             Matrix<N3, N1> visionMeasurementStdDevs) {
//         super(gyroAngle, initialPoseMeters, kinematics, stateStdDevs, localMeasurementStdDevs, visionMeasurementStdDevs);
//         prevPose = initialPoseMeters;
//     }



//     public Pose2d updateWithTime(
//       double currentTimeSeconds, Rotation2d gyroAngle, SwerveModuleState... moduleStates) {
//           prevPose.
//           Pose2d pose = super.updateWithTime(currentTimeSeconds, gyroAngle, moduleStates);
//     }

//     public void resetPosition(Pose2d poseMeters, Rotation2d gyroAngle) {
//         prevPose = poseMeters;
//         super.resetPosition(poseMeters, gyroAngle);
//     }

// }