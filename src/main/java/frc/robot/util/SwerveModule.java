package frc.robot.util;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.MotorPositionSystem.MotorPositionSystemBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class SwerveModule implements Sendable {
  private HSFalcon rotation;
  private HSFalcon drive;

  private CANCoder canCoder;

  private int swerveID;

  private MotorPositionSystem rotationSystem;
  // private MotorVelocitySystem driveSystem;

  private static final double ROTATION_MOTOR_CURRENT_CONTINUOUS = 25;
  private static final double ROTATION_MOTOR_CURRENT_PEAK = 40;
  private static final double ROTATION_MOTOR_CURRENT_PEAK_DUR = 0.1;

  private static final double DRIVE_MOTOR_CURRENT_CONTINUOUS = 30;
  private static final double DRIVE_MOTOR_CURRENT_PEAK = 60;
  private static final double DRIVE_MOTOR_CURRENT_PEAK_DUR = 0.1;

  private static final double DRIVE_kS = 0.2;
  private static final double DRIVE_kV = 2.2819;
  private static final double DRIVE_kA = 0.3621;

  private static final double DRIVE_kP = 0.1; // TODO: Tune
  private static final double DRIVE_kI = 0.0;
  private static final double DRIVE_kD = 0.0;

  private static final double ROTATION_kS = 0;//0.40104;
  private static final double ROTATION_kV = 0.0057859;
  private static final double ROTATION_kA = 0.00016558;

  // private static final double ROTATION_kP = 0.3;
  // private static final double ROTATION_kI = 0.0;
  // private static final double ROTATION_kD = 0.00;

  // private static final double DRIVE_MAX_ERROR = 2;

  private static final double ROTATION_MAX_VEL_ERROR = 0.4;
  private static final double ROTATION_MAX_POS_ERROR = 0.5;

  private static final double WHEEL_DIAMETER = 3.872;
  private static final double ROTATION_GEAR_RATIO = 12.8;
  private static final double DRIVE_GEAR_RATIO = 6.75;
  private static final double DRIVE_FALCON_TO_MPS =
      Conversions.ENCODER_TO_WHEEL_SPEED / DRIVE_GEAR_RATIO * WHEEL_DIAMETER;
  private static final double ROT_MOTOR_TO_DEG = Conversions.ENCODER_TO_DEG / ROTATION_GEAR_RATIO;

  private static final SimpleMotorFeedforward FEEDFORWARD =
      new SimpleMotorFeedforward(DRIVE_kS, DRIVE_kV, DRIVE_kA);
  // private static final SimpleMotorFeedforward ROTATION_FEEDFORWARD = new
  // SimpleMotorFeedforward(ROTATION_kS, ROTATION_kV, ROTATION_kA);

  private static final PIDController PID = new PIDController(DRIVE_kP, DRIVE_kI, DRIVE_kD);
  // private static final PIDController ROTATION_PID = new PIDController(ROTATION_kP, ROTATION_kI,
  // ROTATION_kD);

  public SwerveModule(int swerveID) {
    this.swerveID = swerveID;
    rotation =
        new HSFalconBuilder()
            .invert(Drivetrain.ROTATION_INVERTS[swerveID])
            .supplyLimit(
                ROTATION_MOTOR_CURRENT_PEAK,
                ROTATION_MOTOR_CURRENT_CONTINUOUS,
                ROTATION_MOTOR_CURRENT_PEAK_DUR)
            .velocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms)
            .build(RobotMap.ROTATION_IDS[swerveID], RobotMap.CANBUS);
    SendableRegistry.addLW(
        rotation, "Drivetrain/" + swerveIDToName(swerveID) + " Module", "Rotation Motor");
    drive =
        new HSFalconBuilder()
            .invert(Drivetrain.DRIVE_INVERTS[swerveID])
            .supplyLimit(
                DRIVE_MOTOR_CURRENT_PEAK,
                DRIVE_MOTOR_CURRENT_CONTINUOUS,
                DRIVE_MOTOR_CURRENT_PEAK_DUR)
            .build(RobotMap.TRANSLATION_IDS[swerveID], RobotMap.CANBUS);
    SendableRegistry.addLW(
        drive, "Drivetrain/" + swerveIDToName(swerveID) + " Module", "Drive Motor");
    canCoder = new CANCoder(RobotMap.CANCODER_IDS[swerveID], RobotMap.CANBUS);
    // driveSystem =
    //     new MotorVelocitySystemBuilder()
    //         .constants(DRIVE_kV, DRIVE_kA, DRIVE_kS)
    //         .unitConversionFactor(DRIVE_FALCON_TO_MPS)
    //         .maxError(DRIVE_MAX_ERROR)
    //         .build(drive)
    //         .init();
    rotationSystem =
        new MotorPositionSystemBuilder()
            .constants(ROTATION_kV, ROTATION_kA, ROTATION_kS)
            .unitConversionFactor(ROT_MOTOR_TO_DEG)
            .maxError(ROTATION_MAX_POS_ERROR, ROTATION_MAX_VEL_ERROR)
            .build(rotation)
            .init();
    rotation.config_kF(RobotMap.SLOT_INDEX, 0);
    // rotationSystem.setkP(0.25);
    SendableRegistry.addLW(
        rotationSystem, "Drivetrain/" + swerveIDToName(swerveID) + " Module", "Rotation System");
    // SendableRegistry.addLW(
    //     driveSystem, "Drivetrain/" + swerveIDToName(swerveID) + " Module", "Drive System");
  }

  public void setAngleAndDrive(double rotationAngle, double driveOutput) {
    rotationSystem.set(rotationAngle);
    // ROTATION_FEEDFORWARD.calculate(rotationAngle)+
    // rotation.setVoltage(ROTATION_PID.calculate(getCurrentAngle(), rotationAngle));
    drive.setVoltage(
        FEEDFORWARD.calculate(driveOutput) + PID.calculate(getCurrentSpeed(), driveOutput));
    // driveSystem.set(driveOutput);
  }

  public double getCurrentAngle() {
    return rotationSystem.getPosition();
  }

  public void zeroDriveEncoders() {
    drive.setSelectedSensorPosition(0);
  }

  public Rotation2d getCurrentRotation() {
    return Rotation2d.fromDegrees(getCurrentAngle());
  }

  public double getCurrentSpeed() {
    return drive.getSelectedSensorVelocity() * DRIVE_FALCON_TO_MPS;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getCurrentSpeed(), getCurrentRotation());
  }

  public void setRotationOffset() {
    double position = canCoder.getAbsolutePosition() - Drivetrain.CANCODER_OFFSETS[swerveID];
    rotation.setSelectedSensorPosition(
        position / ROT_MOTOR_TO_DEG); // DEGREE.to(TALONFX, position * ROTATION_GEAR_RATIO));
    SmartDashboard.putNumber("cancoder", position);
    canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, RobotMap.MAX_CAN_FRAME_PERIOD);
  }

  public HSFalcon getDriveMotor() {
    return drive;
  }

  public HSFalcon getRotationMotor() {
    return rotation;
  }

  public CANCoder getCanCoder() {
    return canCoder;
  }

  public static String swerveIDToName(int swerveID) {
    String output = "";
    if (swerveID < 2) output += "Front ";
    else output += "Back ";
    if (swerveID % 2 == 0) output += "Left";
    else output += "Right";
    return output;
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");
    builder.addDoubleProperty("translation velocity", () -> getCurrentSpeed(), null);
    builder.addDoubleProperty("translation desired velocity", () -> PID.getSetpoint(), null);
    builder.addDoubleProperty("translation error", () -> PID.getPositionError(), null);
    // builder.addDoubleProperty("rotation desired position", ()->ROTATION_PID.getSetpoint(), null);
    // builder.addDoubleProperty("rotation error", ()->ROTATION_PID.getPositionError(), null);
    // builder.addDoubleProperty("rotation position", ()->getCurrentAngle(), null);
  }
}
