package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;

public class SwerveManual extends IndefiniteCommand {
    public SwerveManual()
    {
        addRequirements(Drivetrain.getInstance());
    }

    public void initialize() {
        Drivetrain.getInstance().setDrivetrainOffset();
    }

    public void execute() {
        double vx = 0.5 * OI.getInstance().getDriverGamepad().getLeftX();
        double vy = 0.5 * -OI.getInstance().getDriverGamepad().getLeftY();
        double omega = 0.5 * OI.getInstance().getDriverGamepad().getRightX();
        Drivetrain.getInstance().setAngleAndDrive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(Drivetrain.getInstance().getRobotHeading())));
    }
}
