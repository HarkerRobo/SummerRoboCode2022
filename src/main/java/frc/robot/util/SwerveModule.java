package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.wrappers.HSFalcon;

public class SwerveModule {
    private HSFalcon rotation;
    private HSFalcon drive;

    private CANCoder canCoder;

    private int swerveID;

    private LinearSystemRegulationLoop translationLoop;

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

    private static final double MAX_ERROR = 1;  
    private static final double MODEL_STANDARD_DEVIATION = 0.5;
    private static final double ENCODER_STANDARD_DEVIATION = 0.035;

    public static final double ROTATION_GEAR_RATIO = 12.8;
    public static final double DRIVE_GEAR_RATIO = 6.75;
    
    public SwerveModule(int swerveID) {
        this.swerveID = swerveID;
        rotation = new HSFalcon(RobotMap.ROTATION_IDS[swerveID], RobotMap.CANBUS);
        drive = new HSFalcon(RobotMap.TRANSLATION_IDS[swerveID], RobotMap.CANBUS);
        canCoder = new CANCoder(RobotMap.CANCODER_IDS[swerveID], RobotMap.CANBUS);
        translationLoop = new LinearSystemRegulationLoop(LinearSystemId.identifyVelocitySystem(DRIVE_KV, DRIVE_KA), MODEL_STANDARD_DEVIATION, ENCODER_STANDARD_DEVIATION, MAX_ERROR, RobotMap.MAX_MOTOR_VOLTAGE);
        initMotors();
    }

    private void initMotors() {
        initDriveMotor();
        initRotationMotor();
    }

    private void initDriveMotor() {
        drive.configFactoryDefault();
        drive.setNeutralMode(NeutralMode.Brake);
        drive.setInverted(Drivetrain.DRIVE_INVERTS[swerveID]);
        drive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, DRIVE_MOTOR_CURRENT_CONTINUOUS, DRIVE_MOTOR_CURRENT_PEAK, DRIVE_MOTOR_CURRENT_PEAK_DUR));
        drive.configClosedloopRamp(0.3);
        drive.configOpenloopRamp(0.3);
        drive.configForwardSoftLimitEnable(false);
        drive.configReverseSoftLimitEnable(false);
        drive.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
        drive.configVoltageMeasurementFilter(16);
    }

    private void initRotationMotor() {
        rotation.configFactoryDefault();
        rotation.setNeutralMode(NeutralMode.Brake);
        rotation.setInverted(Drivetrain.ROTATION_INVERTS[swerveID]);
        rotation.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, ROTATION_MOTOR_CURRENT_CONTINUOUS, ROTATION_MOTOR_CURRENT_PEAK, ROTATION_MOTOR_CURRENT_PEAK_DUR));
        rotation.configClosedloopRamp(0.1);
        rotation.configOpenloopRamp(0.1);
        rotation.configForwardSoftLimitEnable(false);
        rotation.configReverseSoftLimitEnable(false);
        rotation.config_kP(RobotMap.DEFAULT_SLOT_ID, ROTATION_KP);
        rotation.config_kI(RobotMap.DEFAULT_SLOT_ID, ROTATION_KI);
        rotation.config_kD(RobotMap.DEFAULT_SLOT_ID, ROTATION_KD);
        
        rotation.config_IntegralZone(RobotMap.DEFAULT_SLOT_ID, ROTATION_IZONE);
        rotation.selectProfileSlot(RobotMap.DEFAULT_SLOT_ID, RobotMap.DEFAULT_LOOP_ID);
        rotation.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_5Ms);
        rotation.configVoltageMeasurementFilter(16);
        rotation.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.DEFAULT_LOOP_ID);
    }

    public void setAngleAndDrive(double rotationAngle, double driveOutput, boolean drivePercentOutput) {
        rotation.set(ControlMode.Position, (rotationAngle * Units.DEGREES_TO_ENCODER_TICKS * ROTATION_GEAR_RATIO));
        if(drivePercentOutput) {
            drive.set(ControlMode.PercentOutput, driveOutput / Drivetrain.MAX_TRANSLATION_VEL);
        }
        else {
            drive.set(ControlMode.PercentOutput, MathUtil.clamp(
                translationLoop.updateAndPredict(driveOutput, getCurrentSpeed())
                    + Math.signum(driveOutput) * DRIVE_KS, -RobotMap.MAX_MOTOR_VOLTAGE, RobotMap.MAX_MOTOR_VOLTAGE) / RobotMap.MAX_MOTOR_VOLTAGE);
        }
    }

    public void setAngleAndDrive(double rotationAngle, double driveOutput) {
        setAngleAndDrive(rotationAngle, driveOutput, false);
    }

    public double getCurrentAngle() {
        return rotation.getSelectedSensorPosition() * Units.ENCODER_TICKS_TO_DEGREES / ROTATION_GEAR_RATIO;
    }

    public double getCurrentSpeed() {
        return drive.getSelectedSensorVelocity() * Units.FALCON_VELOCITY_TO_ROT_PER_SECOND * Units.FOUR_INCH_WHEEL_ROT_TO_METER / DRIVE_GEAR_RATIO;
    }

    public void setRotationOffset() {
        double position = canCoder.getAbsolutePosition() - Drivetrain.CANCODER_OFFSETS[swerveID];
        rotation.setSelectedSensorPosition(position * Units.DEGREES_TO_ENCODER_TICKS * ROTATION_GEAR_RATIO);
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
}
