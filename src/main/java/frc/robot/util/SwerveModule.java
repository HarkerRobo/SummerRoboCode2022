package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.wrappers.HSFalcon;

public class SwerveModule implements Sendable{
    private HSFalcon rotation;
    private HSFalcon drive;

    private CANCoder canCoder;

    private int swerveID;

    private LinearSystemRegulationLoop translationLoop;
    private LinearSystemRegulationLoop rotationLoop;

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

    private static final double DRIVE_KS = 0.3;
	private static final double DRIVE_KV = 2.2819;
	private static final double DRIVE_KA = 0.3621;

    private static final double ROTATION_KS = 0.3;
	private static final double ROTATION_KV = 2.2819;
	private static final double ROTATION_KA = 0.3621;

    private static final double DRIVE_MAX_ERROR = 1;  
    private static final double DRIVE_MODEL_STDDEV = 0.5;
    private static final double DRIVE_ENCODER_STDDEV = 0.035;

    private static final double ROTATION_MAX_ERROR = 1;  
    private static final double ROTATION_MODEL_STDDEV = 0.5;
    private static final double ROTATION_ENCODER_STDDEV = 0.035;

    public static final double ROTATION_GEAR_RATIO = 12.8;
    public static final double DRIVE_GEAR_RATIO = 6.75;
    
    public SwerveModule(int swerveID) {
        this.swerveID = swerveID;
        rotation = new HSFalcon(RobotMap.ROTATION_IDS[swerveID], RobotMap.CANBUS);
        drive = new HSFalcon(RobotMap.TRANSLATION_IDS[swerveID], RobotMap.CANBUS);
        canCoder = new CANCoder(RobotMap.CANCODER_IDS[swerveID], RobotMap.CANBUS);
        translationLoop = new LinearSystemRegulationLoop(LinearSystemId.identifyVelocitySystem(DRIVE_KV, DRIVE_KA), DRIVE_MODEL_STDDEV, DRIVE_ENCODER_STDDEV, DRIVE_MAX_ERROR, RobotMap.MAX_MOTOR_VOLTAGE, DRIVE_KS);
        rotationLoop = new LinearSystemRegulationLoop(LinearSystemId.identifyPositionSystem(ROTATION_KV, ROTATION_KA), ROTATION_MODEL_STDDEV, ROTATION_ENCODER_STDDEV, ROTATION_MAX_ERROR, RobotMap.MAX_MOTOR_VOLTAGE, ROTATION_KS);
        initMotors();
    }

    private void initMotors() {
        HSFalconConfigurator.configure(drive, Drivetrain.DRIVE_INVERTS[swerveID], new double[]{1, DRIVE_MOTOR_CURRENT_CONTINUOUS, DRIVE_MOTOR_CURRENT_PEAK, DRIVE_MOTOR_CURRENT_PEAK_DUR}, false);
        HSFalconConfigurator.configure(rotation, Drivetrain.ROTATION_INVERTS[swerveID], new double[]{1, ROTATION_MOTOR_CURRENT_CONTINUOUS, ROTATION_MOTOR_CURRENT_PEAK, ROTATION_MOTOR_CURRENT_PEAK_DUR}, false);
        rotation.configClosedloopRamp(0.1);
        rotation.configOpenloopRamp(0.1);
        rotation.config_kP(RobotMap.DEFAULT_SLOT_ID, ROTATION_KP);
        rotation.config_kI(RobotMap.DEFAULT_SLOT_ID, ROTATION_KI);
        rotation.config_kD(RobotMap.DEFAULT_SLOT_ID, ROTATION_KD);
        
        rotation.config_IntegralZone(RobotMap.DEFAULT_SLOT_ID, ROTATION_IZONE);
        rotation.selectProfileSlot(RobotMap.DEFAULT_SLOT_ID, RobotMap.DEFAULT_LOOP_ID);
        rotation.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_5Ms);
        rotation.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.DEFAULT_LOOP_ID);
    }

    public void setAngleAndDrive(double rotationAngle, double driveOutput, boolean drivePercentOutput) {
        rotation.set(ControlMode.Position, (rotationAngle * Units.DEGREES_TO_ENCODER_TICKS * ROTATION_GEAR_RATIO));
        rotation.setVoltage(rotationLoop.updateAndPredict(rotationAngle, getCurrentAngle()));
        if(drivePercentOutput) {
            drive.set(ControlMode.PercentOutput, driveOutput / Drivetrain.MAX_TRANSLATION_VEL);
        }
        else {
            drive.setVoltage(translationLoop.updateAndPredict(driveOutput, getCurrentSpeed()));
        }
    }

    public void setAngleAndDrive(double rotationAngle, double driveOutput) {
        setAngleAndDrive(rotationAngle, driveOutput, false);
    }

    public double getCurrentAngle() {
        return rotation.getSelectedSensorPosition() * Units.ENCODER_TICKS_TO_DEGREES / ROTATION_GEAR_RATIO;
    }

    public Rotation2d getCurrentRotation() {
        return Rotation2d.fromDegrees(getCurrentAngle());
    }

    public double getCurrentSpeed() {
        return drive.getSelectedSensorVelocity() * Units.FALCON_VELOCITY_TO_ROT_PER_SECOND * Units.wheelRotsToMeter(4) / DRIVE_GEAR_RATIO;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getCurrentSpeed(), getCurrentRotation());
    }

    public void setRotationOffset() {
        double position = canCoder.getAbsolutePosition() - Drivetrain.CANCODER_OFFSETS[swerveID];
        rotation.setSelectedSensorPosition(position * Units.DEGREES_TO_ENCODER_TICKS * ROTATION_GEAR_RATIO);
    }

    public void update() {
        setAngleAndDrive(rotationLoop.getSetpoint(), translationLoop.getSetpoint());
    }

    public LinearSystemRegulationLoop getTranslationLoop() {
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
        builder.addDoubleProperty("Drive Velocity",  () -> getCurrentSpeed(), null);
        builder.addDoubleProperty("Current Rotation",  () -> getCurrentAngle(), null);
        builder.addDoubleProperty("Drive Voltage",  () -> drive.getMotorOutputVoltage(), null);
        builder.addDoubleProperty("Rotation Voltage", () -> rotation.getMotorOutputVoltage(), null);
        builder.addDoubleProperty("Drive Error", () -> translationLoop.getError(), null);
        builder.addDoubleProperty("Rotation Error", () -> rotation.getClosedLoopError(), null);
    }
}
