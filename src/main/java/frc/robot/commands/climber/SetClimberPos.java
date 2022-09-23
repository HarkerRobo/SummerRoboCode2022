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
    double rightError = position - Climber.getInstance().getRightClimberPos();
    double leftError = position - Climber.getInstance().getLeftClimberPos();
    double output = 0.4;
    if (position < 1) {
      if (!Climber.getInstance().rightLimitSwitch())
        Climber.getInstance().setRightPercentOutput(Math.signum(rightError) * output);
      else Climber.getInstance().setRightPercentOutput(0);
      if (!Climber.getInstance().leftLimitSwitch())
        Climber.getInstance().setLeftPercentOutput(Math.signum(leftError) * output);
      else Climber.getInstance().setLeftPercentOutput(0);
    } else {
      if (Math.abs(rightError) >= Climber.POS_MAX_ERROR) {
        output = 0.25;
        if (rightError < 0) output = 0.4;
        Climber.getInstance().setRightPercentOutput(Math.signum(rightError) * output);
      } else {
        Climber.getInstance().setRightPercentOutput(0);
      }
      if (Math.abs(leftError) >= Climber.POS_MAX_ERROR) {
        output = 0.25;
        if (leftError < 0) output = 0.4;
        Climber.getInstance().setLeftPercentOutput(Math.signum(leftError) * output);
      } else {
        Climber.getInstance().setLeftPercentOutput(0);
      }
    }
  }

  public boolean isFinished() {
    if (position < 1) return Climber.getInstance().limitSwitchHit();
    return Math.abs(Climber.getInstance().getRightClimberPos() - position) < Climber.POS_MAX_ERROR
        && Math.abs(Climber.getInstance().getLeftClimberPos() - position) < Climber.POS_MAX_ERROR;
  }

  public void end(boolean interrupted) {
    Climber.getInstance().turnOffMotors();
  }
}
