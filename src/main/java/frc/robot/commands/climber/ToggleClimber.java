package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;

public class ToggleClimber extends InstantCommand {

  public ToggleClimber() {
    addRequirements(Climber.getInstance());
  }

  @Override
  public void initialize() {
    if (Climber.getInstance().getSolenoidState() == DoubleSolenoid.Value.kReverse)
      Climber.getInstance().setClimberBackward();
    else Climber.getInstance().setClimberForward();
  }
}
