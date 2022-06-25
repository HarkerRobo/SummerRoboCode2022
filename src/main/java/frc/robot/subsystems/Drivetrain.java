package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.SwerveModule;

public class Drivetrain extends SubsystemBase {
    public static Drivetrain instance;

    public static final boolean[] ROTATION_INVERTS = {false, false, false, false};
    public static final boolean[] DRIVE_INVERTS = {true, true, true, false};

    public static final double[] CANCODER_OFFSETS = {195.468750, 178.330078, 109.951172, 32.255859}; // in deg

    private static final double DT_WIDTH = 0.5461; // 0.93345 bumper to bumper
    private static final double DT_LENGTH = 0.5969; // 0.88265

    public static final double MAX_TRANSLATION_VEL =  3.0; // in m/s
    public static final double MAX_ROTATION_VEL = 1.5 * Math.PI; // in rad/s

    private static final double ODOMETRY_MODEL_STDEV = 0.0;
    private static final double ODOMEDTRY_ENCODER_STDEV = 0.0;
    private static final double ODOMETRY_VISION_STDEV = 0.0;

    private SwerveModule[] swerveModules;
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator estimator;
    private Pigeon2 pigeon;

    private Drivetrain() {
        swerveModules = new SwerveModule[]{new SwerveModule(0), new SwerveModule(1), new SwerveModule(2), new SwerveModule(3)};

        kinematics = new SwerveDriveKinematics(new Translation2d(DT_LENGTH/2, DT_WIDTH/2), new Translation2d(DT_LENGTH/2, -DT_WIDTH/2),
            new Translation2d(-DT_LENGTH/2, DT_WIDTH/2), new Translation2d(-DT_LENGTH/2, -DT_WIDTH/2));

        pigeon = new Pigeon2(RobotMap.PIGEON_ID, RobotMap.CANBUS);

        // estimator = new SwerveDrivePoseEstimator(getRobotHeading(), new Pose2d(0, 0, Rotation2d.fromDegrees(getRobotHeading())), kinematics, ODOMETRY_MODEL_STDEV, ODOMEDTRY_ENCODER_STDEV, ODOMETRY_VISION_STDEV);
    }

    public void setAngleAndDrive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        for (int i = 0; i < 4; i++) {
            states[i] = SwerveModuleState.optimize(states[i], Rotation2d.fromDegrees(swerveModules[i].getCurrentAngle()));
            swerveModules[i].setAngleAndDrive(states[i].angle.getDegrees(), states[i].speedMetersPerSecond);
        }
    }

    public void setDrivetrainOffset() {
        for (int i = 0; i < 4; i++)
            swerveModules[i].setRotationOffset();
    }

    public double getRobotHeading() {
        return pigeon.getYaw();
    }

    public static Drivetrain getInstance() {
        if(instance == null) instance = new Drivetrain();
        return instance;
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Drivetrain");
        builder.addDoubleProperty("Swerve Module 0 Drive Error", () -> swerveModules[0].getTranslationLoop().getError(), null);
        builder.addDoubleProperty("Swerve Module 0 Drive Voltage", () -> swerveModules[0].getDriveMotor().getMotorOutputVoltage(), null);
        builder.addDoubleProperty("Swerve Module 0 Rotation Voltage", () -> swerveModules[0].getRotationMotor().getMotorOutputVoltage(), null);
    }
}
