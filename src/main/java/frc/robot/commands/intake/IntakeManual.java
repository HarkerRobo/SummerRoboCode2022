package frc.robot.commands.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;

public class IntakeManual extends IndefiniteCommand{

    private double speed;

    public IntakeManual(double speed) {
        addRequirements(Intake.getInstance());
        this.speed = speed;
    }

    public void execute() {

        Intake.getInstance().setRollerOutput(speed / Intake.MAX_ROLLER_SPEED);
    }

    public void end() {
        Intake.getInstance().setRollerOutput(0);
    }

}
