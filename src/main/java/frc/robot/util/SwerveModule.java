package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import harkerrobolib.wrappers.HSFalcon;

public class SwerveModule {
    private HSFalcon turn;
    private HSFalcon drive;
    private static final double ANGLE_MOTOR_CURRENT_CONTINUOUS = 25;
    private static final double ANGLE_MOTOR_CURRENT_PEAK = 40;
    private static final double ANGLE_MOTOR_CURRENT_PEAK_DUR = 0.1;

	private static final double DRIVE_MOTOR_CURRENT_CONTINUOUS = 35;
    private static final double DRIVE_MOTOR_CURRENT_PEAK = 60;
    private static final double DRIVE_MOTOR_CURRENT_PEAK_DUR = 0.1;
    
    public SwerveModule(int turnID, int driveID) {
        turn = new HSFalcon(turnID);
        drive = new HSFalcon(driveID);
    }

    public void initMotors() {
        turn.configFactoryDefault();
        drive.configFactoryDefault();
        turn.setNeutralMode(NeutralMode.Brake);
        drive.setNeutralMode(NeutralMode.Brake);

    }

    public void setAngleAndDrive(double turnOutput, double driveOutput) {
        turn.set(ControlMode.PercentOutput, turnOutput);
        drive.set(ControlMode.PercentOutput, driveOutput);
    }
}
