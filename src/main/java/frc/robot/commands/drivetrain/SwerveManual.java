package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

import frc.robot.subsystems.Shooter;
import frc.robot.util.Constants;
import frc.robot.util.PhotonVisionLimelight;
// import frc.robot.util.SwerveControllerLoop;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

public class SwerveManual extends IndefiniteCommand {
  public static final double SPEED_MULTIPLIER = 1;
  public static final double MIN_OUTPUT = 0.0001;
  private static final double PIGEON_KP = 0.07;
  private static final double PIGEON_DELAY = 0.4;
  private static final double MAX_ACCELERATION = 0.8/RobotMap.ROBOT_LOOP;

  public static final double LIMELIGHT_KP = 0.07;
  public static final double LIMELIGHT_KI = 0.01;
  public static final double LIMELIGHT_KD = 0.00000;
  private Debouncer debouncer;

  private double vx;
  private double prevvx;
  private double vy;
  private double prevvy;
  private double omega;
  private double prevomega;

  private double pigeonAngle;
  private boolean holdingPigeonAngle;

  private boolean rotationOutput;
  private boolean translationOutput;
  private boolean aligningWithHub;

  private static ProfiledPIDController HUB_LOOP = new ProfiledPIDController(LIMELIGHT_KP, LIMELIGHT_KI, LIMELIGHT_KD, new Constraints(4, 3.5));

  // private static SwerveControllerLoop HUB_LOOP = new SwerveControllerLoop();

  public SwerveManual() {
    addRequirements(Drivetrain.getInstance());
    vx = vy = omega = prevvx = prevvy = prevomega = 0;
    aligningWithHub = false;
    holdingPigeonAngle = false;
    debouncer = new Debouncer(PIGEON_DELAY, DebounceType.kRising);
    pigeonAngle = Drivetrain.getInstance().getRobotHeading();
    HUB_LOOP.setGoal(0);
  }

  public void initialize() {
    Drivetrain.getInstance().setDrivetrainOffset();
  }

  public void execute() {
    prevvx = vx;
    prevvy = vy;
    prevomega = omega;
    vx =
        MathUtil.mapJoystickOutput(
            OI.getInstance().getDriverGamepad().getLeftX(), OI.DEFAULT_DEADBAND);
    vy =
        MathUtil.mapJoystickOutput(
            OI.getInstance().getDriverGamepad().getLeftY(), OI.DEFAULT_DEADBAND);
    omega =
        -MathUtil.mapJoystickOutput(
            OI.getInstance().getDriverGamepad().getRightX(), OI.DEFAULT_DEADBAND);
    squareInputs();
    scaleToDrivetrainSpeeds();
    if(omega < MIN_OUTPUT && Math.sqrt(vx * vx + vy * vy) > MIN_OUTPUT) {
      adjustPigeon();
    }
    else {holdingPigeonAngle = false;}
    if (Shooter.getInstance().getState() != Shooter.State.IDLE && omega <= MIN_OUTPUT)
      alignWithHub();
    if (Math.abs(omega) <= MIN_OUTPUT && Math.sqrt(vx * vx + vy * vy) <= MIN_OUTPUT) {
      omega = MIN_OUTPUT;
      ChassisSpeeds speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              vx, vy, omega, Drivetrain.getInstance().getRobotRotation());
      var states = Drivetrain.getInstance().getKinematics().toSwerveModuleStates(speeds);
      // for (int i = 0; i < 4; i++) states[i].angle = states[i].angle.plus(Rotation2d.fromDegrees(90));
      Drivetrain.getInstance().setAngleAndDrive(states);
    } else {
      Drivetrain.getInstance()
          .setAngleAndDrive(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  vx,
                  vy,
                  omega,
                  Drivetrain.getInstance().getRobotRotation()));
    }
  }

  public double magnitude(double x, double y) {
    return Math.sqrt(x*x + y*y);
  }

  public void limitAcceleration() {
    double mag = magnitude(vx-prevvx, vy-prevvy)/RobotMap.ROBOT_LOOP;
    if(mag > MAX_ACCELERATION) {
      double scale = mag/MAX_ACCELERATION;
      vx += (prevvx-vx) * scale;
      vy += (prevvy-vy) * scale;
    }
  }

  public void adjustPigeon() {
    if(!holdingPigeonAngle) {
      holdingPigeonAngle = true;
      pigeonAngle = Drivetrain.getInstance().getRobotHeading();
    }
    omega = -PIGEON_KP * (Drivetrain.getInstance().getRobotHeading() - pigeonAngle);
  }

  public void alignWithHub() {
    double angleToHub = PhotonVisionLimelight.dx;
    omega = HUB_LOOP.calculate(angleToHub);
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
