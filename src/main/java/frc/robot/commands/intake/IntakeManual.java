package frc.robot.commands.intake;

import frc.robot.OI;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;

public class IntakeManual extends IndefiniteCommand{

    private static final double INTAKE_SPEED = 36.0;

    public IntakeManual() {
        addRequirements(Intake.getInstance());
    }

    public void execute() {
        if (OI.getInstance().getDriverGamepad().getRightTrigger() > OI.DEFAULT_DEADBAND) {
            Intake.getInstance().setRollerOutput(INTAKE_SPEED);
            Intake.getInstance().setCurrIntakeState(Intake.State.INTAKE);
        }
        else if (OI.getInstance().getDriverGamepad().getLeftTrigger() > OI.DEFAULT_DEADBAND) {
            Intake.getInstance().setRollerOutput(-INTAKE_SPEED);
            Intake.getInstance().setCurrIntakeState(Intake.State.OUTTAKE);
        }
        else {
            Intake.getInstance().setRollerOutput(0);
            Intake.getInstance().setCurrIntakeState(Intake.State.NEUTRAL);
        }
    }

    public void end(boolean interrupted) {
        Intake.getInstance().setCurrIntakeState(Intake.State.NEUTRAL);
        Intake.getInstance().setRollerOutput(0);
    }

}
