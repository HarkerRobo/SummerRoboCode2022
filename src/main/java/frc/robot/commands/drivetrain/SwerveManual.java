package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.OI;
import frc.robot.commands.auton.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.util.FieldConstants;
import frc.robot.util.loop.PositionControlLoop;
import frc.robot.util.loop.PositionControlLoop.PositionControlLoopBuilder;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

public class SwerveManual extends IndefiniteCommand {
    public static final double SPEED_MULTIPLIER = 0.6;

    private double vx;
    private double vy;
    private double omega;

    private boolean aligningWithHub;

    private static final double THETA_MAX_POS_ERROR = 5; //TODO
    private static final double THETA_MAX_VEL_ERROR = 5;   //TODO
    private static final double THETA_MODEL_POS_STDEV = 0.5; //TODO
    private static final double THETA_MODEL_VEL_STDEV = 0.5; //TODO
    private static final double THETA_MEAS_STDEV = 0.035; //TODO
    

    private static PositionControlLoop HUB_LOOP = new PositionControlLoopBuilder()
                                                    .stateMatrices(SwerveControllerCommand.A, SwerveControllerCommand.B, 
                                                        SwerveControllerCommand.C, SwerveControllerCommand.D)
                                                    .standardDeviations(THETA_MODEL_POS_STDEV, THETA_MODEL_VEL_STDEV, THETA_MEAS_STDEV)
                                                    .maxError(THETA_MAX_POS_ERROR, THETA_MAX_VEL_ERROR)
                                                    .maxControlEffort(SPEED_MULTIPLIER * Drivetrain.MAX_ROTATION_VEL)
                                                    .buildPositionControlLoop(); 

    public SwerveManual() {
        addRequirements(Drivetrain.getInstance());
        vx = vy = omega = 0;
        aligningWithHub = false;
    }

    public void initialize() {
        Drivetrain.getInstance().setDrivetrainOffset();
    }

    public void execute() {
        vx = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.DEFAULT_DEADBAND);
        vy = -MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.DEFAULT_DEADBAND);
        omega = -MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEFAULT_DEADBAND);
        squareInputs();
        scaleToDrivetrainSpeeds();
        if(Shooter.getInstance().getState() != Shooter.State.IDLE) alignWithHub();
        else aligningWithHub = false;
        Drivetrain.getInstance().setAngleAndDrive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(Drivetrain.getInstance().getRobotHeading())));
    }

    public void alignWithHub() {
        if(!aligningWithHub)
            HUB_LOOP.reset(Drivetrain.getInstance().getRobotHeading(), Drivetrain.getInstance().getChassisSpeeds().omegaRadiansPerSecond);
        Translation2d diff = FieldConstants.HUB_LOCATION.minus(Drivetrain.getInstance().getPoseEstimator().getEstimatedPosition().getTranslation());
        double angleToHub = Math.toDegrees(Math.atan2(diff.getY(), diff.getX()));
        omega = HUB_LOOP.setReferenceAndPredict(angleToHub, 0.0, Drivetrain.getInstance().getRobotHeading());
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
