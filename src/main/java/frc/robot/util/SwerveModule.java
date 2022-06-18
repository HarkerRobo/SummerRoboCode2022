package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.CANCoder;

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
    
    public SwerveModule(int swerveID) {
        this.swerveID = swerveID;
        rotation = new HSFalcon(RobotMap.ROTATION_IDS[swerveID], RobotMap.CANBUS);
        drive = new HSFalcon(RobotMap.TRANSLATION_IDS[swerveID], RobotMap.CANBUS);
        canCoder = new CANCoder(RobotMap.CANCODER_IDS[swerveID], RobotMap.CANBUS);
    }

    public void initMotors() {
        rotation.configFactoryDefault();
        rotation.setNeutralMode(NeutralMode.Brake);
        rotation.setInverted(Drivetrain.ROTATION_INVERTS[swerveID]);
        rotation.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, ROTATION_MOTOR_CURRENT_CONTINUOUS, ROTATION_MOTOR_CURRENT_PEAK, ROTATION_MOTOR_CURRENT_PEAK_DUR));
        rotation.configOpenloopRamp(0.1);
        rotation.configClosedloopRamp(0.1);
        rotation.configForwardSoftLimitEnable(false);
        rotation.configReverseSoftLimitEnable(false);
        rotation.config_kP(arg0, arg1)

        drive.configFactoryDefault();
        drive.setNeutralMode(NeutralMode.Brake);
        drive.setInverted(Drivetrain.DRIVE_INVERTS[swerveID]);
        drive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, DRIVE_MOTOR_CURRENT_CONTINUOUS, DRIVE_MOTOR_CURRENT_PEAK, DRIVE_MOTOR_CURRENT_PEAK_DUR));
        drive.configForwardSoftLimitEnable(false);
        drive.configReverseSoftLimitEnable(false);
    }

    public void setAngleAndDrive(double rotationOutput, double driveOutput) {
        rotation.set(ControlMode.PercentOutput, rotationOutput);
        drive.set(ControlMode.PercentOutput, driveOutput);
    }
}
