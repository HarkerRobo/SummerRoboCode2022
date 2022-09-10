package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Constants;
import frc.robot.util.SwerveControllerLoop;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

public class SwerveManual extends IndefiniteCommand {
  public static final double SPEED_MULTIPLIER = 0.6;

  private double vx;
  private double vy;
  private double omega;

  private boolean aligningWithHub;

  private static SwerveControllerLoop HUB_LOOP = new SwerveControllerLoop();

  public SwerveManual() {
    addRequirements(Drivetrain.getInstance());
    vx = vy = omega = 0;
    aligningWithHub = false;
  }

  public void initialize() {
    Drivetrain.getInstance().setDrivetrainOffset();
  }

  public void execute() {
    vx =
        MathUtil.mapJoystickOutput(
            OI.getInstance().getDriverGamepad().getLeftY(), OI.DEFAULT_DEADBAND);
    vy =
        -MathUtil.mapJoystickOutput(
            OI.getInstance().getDriverGamepad().getLeftX(), OI.DEFAULT_DEADBAND);
    omega =
        -MathUtil.mapJoystickOutput(
            OI.getInstance().getDriverGamepad().getRightX(), OI.DEFAULT_DEADBAND);
    squareInputs();
    scaleToDrivetrainSpeeds();
    if (Shooter.getInstance().getState() != Shooter.State.IDLE) alignWithHub();
    else aligningWithHub = false;
    Drivetrain.getInstance()
        .setAngleAndDrive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                vx, vy, omega, Rotation2d.fromDegrees(Drivetrain.getInstance().getRobotHeading())));
  }

  public void alignWithHub() {
    Translation2d diff =
        Constants.HUB_LOCATION.minus(
            Drivetrain.getInstance().getPoseEstimator().getEstimatedPosition().getTranslation());
    double angleToHub = Math.toDegrees(Math.atan2(diff.getY(), diff.getX()));
  }

  public void squareInputs() {
    vx *= Math.abs(vx);
    vy *= Math.abs(vy);
    omega *= Math.abs(omega);
  }

  public void scaleToDrivetrainSpeeds() {
    vx *= SPEED_MULTIPLIER * Drivetrain.MAX_TRANSLATION_VEL;
    vy *= SPEED_MULTIPLIER * Drivetrain.MAX_TRANSLATION_VEL;
    omega *= SPEED_MULTIPLIER * Drivetrain.MAX_ROTATION_VEL;
  }
}
