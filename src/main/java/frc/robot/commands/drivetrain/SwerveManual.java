package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.util.PhotonVisionLimelight;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

public class SwerveManual extends IndefiniteCommand {
  public static final double SPEED_MULTIPLIER = 1;
  public static final double MIN_OUTPUT = 0.0001;
  private static final double PIGEON_KP = 0.07;
  private static final double MAX_ACCELERATION = Drivetrain.MAX_ACCELERATION;

  public static final double LIMELIGHT_KP = 0.07;
  public static final double LIMELIGHT_KI = 0.01;
  public static final double LIMELIGHT_KD = 0.00000;

  private double vx;
  private double prevvx;
  private double vy;
  private double prevvy;
  private double omega;

  private double pigeonAngle;
  private boolean holdingPigeonAngle;

  private boolean aligningWithHub;

  private static ProfiledPIDController HUB_LOOP =
      new ProfiledPIDController(LIMELIGHT_KP, LIMELIGHT_KI, LIMELIGHT_KD, new Constraints(4, 3.5));

  // private static SwerveControllerLoop HUB_LOOP = new SwerveControllerLoop();

  public SwerveManual() {
    addRequirements(Drivetrain.getInstance());
    vx = vy = omega = prevvx = prevvy = 0;
    aligningWithHub = false;
    holdingPigeonAngle = false;
    pigeonAngle = Drivetrain.getInstance().getRobotHeading();
    HUB_LOOP.setGoal(0);
  }

  public void initialize() {
    Drivetrain.getInstance().setDrivetrainOffset();
  }

  public void execute() {
    prevvx = vx;
    prevvy = vy;
    vx =
        -MathUtil.mapJoystickOutput(
            OI.getInstance().getDriverGamepad().getLeftY(), OI.DEFAULT_DEADBAND);
    vy =
        MathUtil.mapJoystickOutput(
            OI.getInstance().getDriverGamepad().getLeftY(), OI.DEFAULT_DEADBAND);
    omega =
        -MathUtil.mapJoystickOutput(
            OI.getInstance().getDriverGamepad().getRightX(), OI.DEFAULT_DEADBAND);
    squareInputs();
    scaleToDrivetrainSpeeds();
    if (omega < MIN_OUTPUT && magnitude(vx, vy) > MIN_OUTPUT) {
      adjustPigeon();
    } else {
      holdingPigeonAngle = false;
    }
    if (Shooter.getInstance().getState() != Shooter.State.IDLE && omega <= MIN_OUTPUT)
      alignWithHub();
    if (Math.abs(omega) <= MIN_OUTPUT && magnitude(vx, vy) <= MIN_OUTPUT) {
      omega = MIN_OUTPUT;
      vx = vy = 0;
      // ChassisSpeeds speeds =
      //     ChassisSpeeds.fromFieldRelativeSpeeds(
      //         vx, vy, omega, Drivetrain.getInstance().getRobotRotation());
      // var states = Drivetrain.getInstance().getKinematics().toSwerveModuleStates(speeds);
      // for (int i = 0; i < 4; i++) states[i].angle =
      // states[i].angle.plus(Rotation2d.fromDegrees(90));
      // Drivetrain.getInstance().setAngleAndDrive(states);
    }
    Drivetrain.getInstance()
        .setAngleAndDrive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                vx, vy, omega, Drivetrain.getInstance().getRobotRotation()));
  }

  private double magnitude(double x, double y) {
    return Math.sqrt(x * x + y * y);
  }

  public void limitAcceleration() {
    double magAcc = magnitude(vx - prevvx, vy - prevvy) / RobotMap.ROBOT_LOOP;
    if (magAcc > MAX_ACCELERATION) {
      double scale = MAX_ACCELERATION / magAcc;
      vx += (vx - prevvx) / RobotMap.ROBOT_LOOP * scale;
      vy += (vy - prevvy) / RobotMap.ROBOT_LOOP * scale;
    }
  }

  public void adjustPigeon() {
    if (!holdingPigeonAngle) {
      holdingPigeonAngle = true;
      pigeonAngle = Drivetrain.getInstance().getRobotHeading();
    }
    omega = -PIGEON_KP * (Drivetrain.getInstance().getRobotHeading() - pigeonAngle);
  }

  public void alignWithHub() {
    double angleToHub = PhotonVisionLimelight.getTx(); // cw positive
    omega = -HUB_LOOP.calculate(angleToHub);
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
