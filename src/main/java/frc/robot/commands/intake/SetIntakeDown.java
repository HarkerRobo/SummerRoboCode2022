package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class SetIntakeDown extends InstantCommand {
    public SetIntakeDown() {
        addRequirements(Intake.getInstance());
    }

    public void initialize() {
        Intake.getInstance().setBackward();
    }
    
}
