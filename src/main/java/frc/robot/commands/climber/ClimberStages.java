// package frc.robot.commands.climber;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.OI;
// import frc.robot.subsystems.Climber;

// public class ClimberStages {
//   private static class WaitForDriver extends WaitUntilCommand {
//     public WaitForDriver() {
//       super(() -> OI.getInstance().getDriverGamepad().getButtonAState());
//     }
//   }

//   public static final SequentialCommandGroup STAGE_1 =
//       new SequentialCommandGroup(
//           new SetClimberPos(Climber.UP_HEIGHT),
//           new WaitForDriver(),
//           new SetClimberPos(Climber.DOWN_HEIGHT));

//   public static final SequentialCommandGroup STAGE_2_AND_3 =
//       new SequentialCommandGroup(
//           new SetClimberPos(Climber.MID_HEIGHT),
//           new InstantCommand(() -> Climber.getInstance().setClimberForward()),
//           new SetClimberPos(Climber.UP_HEIGHT),
//           new WaitForDriver(),
//           new InstantCommand(() -> Climber.getInstance().setClimberBackward()),
//           new WaitForDriver(),
//           new SetClimberPos(Climber.DOWN_HEIGHT));

//   public static final SequentialCommandGroup ALL_STAGES =
//       new SequentialCommandGroup(
//           STAGE_1, new WaitForDriver(), STAGE_2_AND_3, new WaitForDriver(), STAGE_2_AND_3);
// }
