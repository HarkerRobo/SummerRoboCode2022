package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

public class SwerveManual extends IndefiniteCommand {
    public static final double SPEED_MULTIPLIER = 0.6;

    public SwerveManual() {
        addRequirements(Drivetrain.getInstance());
    }

    public void initialize() {
        Drivetrain.getInstance().setDrivetrainOffset();
    }

    public void execute() {
        double vx = Drivetrain.MAX_TRANSLATION_VEL * -MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.DEFAULT_DEADBAND);
        double vy = Drivetrain.MAX_TRANSLATION_VEL * -MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.DEFAULT_DEADBAND);
        double omega = Drivetrain.MAX_ROTATION_VEL * -MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEFAULT_DEADBAND);
        vx *= SPEED_MULTIPLIER;
        vy *= SPEED_MULTIPLIER;
        omega *= SPEED_MULTIPLIER;
        Drivetrain.getInstance().setAngleAndDrive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(Drivetrain.getInstance().getRobotHeading())));
    }
}
