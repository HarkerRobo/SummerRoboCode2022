package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.loop.PositionControlLoop;
import frc.robot.util.loop.VelocityControlLoop;
import harkerrobolib.wrappers.HSFalcon;

public class SwerveModule implements Sendable {
  private HSFalcon rotation;
  private HSFalcon drive;

  private CANCoder canCoder;

  private int swerveID;

  private VelocityControlLoop translationLoop;
  private PositionControlLoop rotationLoop;

  private static final double ROTATION_MOTOR_CURRENT_CONTINUOUS = 25;
  private static final double ROTATION_MOTOR_CURRENT_PEAK = 40;
  private static final double ROTATION_MOTOR_CURRENT_PEAK_DUR = 0.1;

  private static final double DRIVE_MOTOR_CURRENT_CONTINUOUS = 35;
  private static final double DRIVE_MOTOR_CURRENT_PEAK = 60;
  private static final double DRIVE_MOTOR_CURRENT_PEAK_DUR = 0.1;

  public static final double ROTATION_KP = 0.3;
  public static final double ROTATION_KI = 0.0;
  public static final double ROTATION_KD = 0.0;
  public static final double ROTATION_IZONE = 0.0;

  private static final double DRIVE_kS = 0.3;
  private static final double DRIVE_kV = 2.2819;
  private static final double DRIVE_kA = 0.3621;

  private static final double ROTATION_kS = 0.3;
  private static final double ROTATION_kV = 2.2819;
  private static final double ROTATION_kA = 0.3621;

  private static final double DRIVE_MAX_ERROR = 1;
  private static final double DRIVE_MODEL_STDEV = 0.5;
  private static final double DRIVE_ENCODER_STDEV = 0.035;

  private static final double ROTATION_MAX_VEL_ERROR = 1;
  private static final double ROTATION_MAX_POS_ERROR = 1;
  private static final double ROTATION_POS_MODEL_STDEV = 0.5;
  private static final double ROTATION_VEL_MODEL_STDEV = 0.5;
  private static final double ROTATION_ENCODER_STDEV = 0.035;

  public static final double ROTATION_GEAR_RATIO = 12.8;
  public static final double DRIVE_GEAR_RATIO = 6.75;

  public SwerveModule(int swerveID) {
    this.swerveID = swerveID;
    rotation =
        new HSFalconBuilder()
            .invert(Drivetrain.ROTATION_INVERTS[swerveID])
            .supplyLimit(
                ROTATION_MOTOR_CURRENT_PEAK,
                ROTATION_MOTOR_CURRENT_CONTINUOUS,
                ROTATION_MOTOR_CURRENT_PEAK_DUR)
            .velocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_5Ms)
            .build(RobotMap.ROTATION_IDS[swerveID], RobotMap.CANBUS);
    drive =
        new HSFalconBuilder()
            .invert(Drivetrain.DRIVE_INVERTS[swerveID])
            .supplyLimit(
                DRIVE_MOTOR_CURRENT_PEAK,
                DRIVE_MOTOR_CURRENT_CONTINUOUS,
                DRIVE_MOTOR_CURRENT_PEAK_DUR)
            .build(RobotMap.TRANSLATION_IDS[swerveID], RobotMap.CANBUS);
    canCoder = new CANCoder(RobotMap.CANCODER_IDS[swerveID], RobotMap.CANBUS);
    translationLoop =
        new VelocityControlLoop.VelocityControlLoopBuilder()
            .motorConstants(DRIVE_kS, DRIVE_kA, DRIVE_kV)
            .standardDeviations(DRIVE_MODEL_STDEV, DRIVE_ENCODER_STDEV)
            .maxError(DRIVE_MAX_ERROR)
            .buildVelocityControlLoop();
    rotationLoop =
        new PositionControlLoop.PositionControlLoopBuilder()
            .motorConstants(ROTATION_kS, ROTATION_kA, ROTATION_kV)
            .standardDeviations(
                ROTATION_POS_MODEL_STDEV, ROTATION_VEL_MODEL_STDEV, ROTATION_ENCODER_STDEV)
            .maxError(ROTATION_MAX_POS_ERROR, ROTATION_MAX_VEL_ERROR)
            .buildPositionControlLoop();
  }

  public void setAngleAndDrive(
      double rotationAngle, double driveOutput, boolean drivePercentOutput) {
    // rotation.set(ControlMode.Position, (rotationAngle * Units.DEGREES_TO_ENCODER_TICKS *
    // ROTATION_GEAR_RATIO));
    rotation.setVoltage(rotationLoop.setReferenceAndPredict(rotationAngle, 0.0, getCurrentAngle()));
    if (drivePercentOutput) {
      drive.set(ControlMode.PercentOutput, driveOutput / Drivetrain.MAX_TRANSLATION_VEL);
    } else {
      drive.setVoltage(translationLoop.setReferenceAndPredict(driveOutput, getCurrentSpeed()));
    }
  }

  public void setAngleAndDrive(double rotationAngle, double driveOutput) {
    setAngleAndDrive(rotationAngle, driveOutput, false);
  }

  public double getCurrentAngle() {
    return rotation.getSelectedSensorPosition()
        * Units.ENCODER_TICKS_TO_DEGREES
        / ROTATION_GEAR_RATIO;
  }

  public double getCurrentAngleVelocity() {
    return rotation.getSelectedSensorVelocity()
        * Units.ENCODER_TICKS_TO_DEGREES
        / ROTATION_GEAR_RATIO;
  }

  public Rotation2d getCurrentRotation() {
    return Rotation2d.fromDegrees(getCurrentAngle());
  }

  public double getCurrentSpeed() {
    return drive.getSelectedSensorVelocity()
        * Units.FALCON_VELOCITY_TO_ROT_PER_SECOND
        * Units.wheelRotsToMeter(4.0)
        / DRIVE_GEAR_RATIO;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getCurrentSpeed(), getCurrentRotation());
  }

  public void setRotationOffset() {
    double position = canCoder.getAbsolutePosition() - Drivetrain.CANCODER_OFFSETS[swerveID];
    rotation.setSelectedSensorPosition(
        position * Units.DEGREES_TO_ENCODER_TICKS * ROTATION_GEAR_RATIO);
    canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, RobotMap.MAX_CAN_FRAME_PERIOD);
  }

  public void update() {
    setAngleAndDrive(rotationLoop.getFilteredPosition(), translationLoop.getSetpoint());
  }

  public VelocityControlLoop getTranslationLoop() {
    return translationLoop;
  }

  public HSFalcon getDriveMotor() {
    return drive;
  }

  public HSFalcon getRotationMotor() {
    return rotation;
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Swerve Module " + swerveID);
    builder.addDoubleProperty("Drive Velocity", () -> getCurrentSpeed(), null);
    builder.addDoubleProperty("Current Rotation", () -> getCurrentAngle(), null);
    builder.addDoubleProperty("Drive Voltage", () -> drive.getMotorOutputVoltage(), null);
    builder.addDoubleProperty("Rotation Voltage", () -> rotation.getMotorOutputVoltage(), null);
    builder.addDoubleProperty(
        "Drive Error",
        () -> translationLoop.getFilteredVelocity() - translationLoop.getSetpoint(),
        null);
    builder.addDoubleProperty("Rotation Error", () -> rotation.getClosedLoopError(), null);
  }
}
