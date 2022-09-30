package frc.robot.commands.auton;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;

// public class SwervePosController extends SwerveControllerCommand{
//     private PIDController xController;
//     private PIDController yController;
//     private ProfiledPIDController thetaController;
//     private 
//     public SwervePosController(Trajectory trajectory,
//     Supplier<Pose2d> pose,
//     SwerveDriveKinematics kinematics) {
//         super(trajectory, pose, kinematics, xController, yController, thetaController, Drivetrain.getInstance()::setAngleAndDrive, Drivetrian.getInstance());
//     }
    
// }
