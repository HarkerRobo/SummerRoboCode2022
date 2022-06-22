package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.wrappers.HSFalcon;

public class SwerveModule {
    private HSFalcon rotation;
    private HSFalcon drive;

    private CANCoder canCoder;


    private int swerveID;

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

    public static final double ROTATION_GEAR_RATIO = 12.8;
    public static final double DRIVE_GEAR_RATIO = 6.75;
    
    public SwerveModule(int swerveID) {
        this.swerveID = swerveID;
        rotation = new HSFalcon(RobotMap.ROTATION_IDS[swerveID], RobotMap.CANBUS);
        drive = new HSFalcon(RobotMap.TRANSLATION_IDS[swerveID], RobotMap.CANBUS);
        canCoder = new CANCoder(RobotMap.CANCODER_IDS[swerveID], RobotMap.CANBUS);
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
        rotation.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.DEFAULT_SLOT_ID);
    }

    public void setAngleAndDrive(double rotationAngle, double driveOutput, boolean drivePercentOutput) {
        rotation.set(ControlMode.Position, (rotationAngle * Units.DEGREES_TO_ENCODER_TICKS * ROTATION_GEAR_RATIO));
        if(drivePercentOutput) {

            drive.set(ControlMode.PercentOutput, driveOutput / Drivetrain.MAX_TRANSLATION_VEL * Drivetrain.SPEED_MULTIPLIER);
        }
            
        else {
            // TODO
        }
    }

    public void setAngleAndDrive(double rotationAngle, double driveOutput) {
        setAngleAndDrive(rotationAngle, driveOutput, false);
    }

    public double getCurrentAngle() {
        return rotation.getSelectedSensorPosition() * Units.ENCODER_TICKS_TO_DEGREES / ROTATION_GEAR_RATIO;
    }

    public void setRotationOffset() {
        double position = canCoder.getAbsolutePosition() - Drivetrain.CANCODER_OFFSETS[swerveID];
        rotation.setSelectedSensorPosition(position * Units.DEGREES_TO_ENCODER_TICKS * ROTATION_GEAR_RATIO);
    }

}
