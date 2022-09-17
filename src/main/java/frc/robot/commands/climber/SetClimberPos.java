package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class SetClimberPos extends CommandBase {
  private double position;

  public SetClimberPos(double pos) {
    addRequirements(Climber.getInstance());
    position = pos;
  }

  public void execute() {
    if(Math.abs(Climber.getInstance().getRightClimberPos() - position) >= Climber.POS_MAX_ERROR) {
      Climber.getInstance().setRightPercentOutput(-Math.signum(Climber.getInstance().getRightClimberPos() - position) * 0.25);
    }
    else {
      Climber.getInstance().setRightPercentOutput(0);
    }
    if(Math.abs(Climber.getInstance().getLeftClimberPos() - position) >= Climber.POS_MAX_ERROR) {
      Climber.getInstance().setLeftPercentOutput(-Math.signum(Climber.getInstance().getLeftClimberPos() - position) * 0.25);
    }
    else {
      Climber.getInstance().setLeftPercentOutput(0);
    }
  }

  public boolean isFinished() {
    return Math.abs(Climber.getInstance().getRightClimberPos() - position) < Climber.POS_MAX_ERROR && Math.abs(Climber.getInstance().getLeftClimberPos() - position) < Climber.POS_MAX_ERROR;
  }

  public void end(boolean interrupted) {
    Climber.getInstance().turnOffMotors();
  }
}
