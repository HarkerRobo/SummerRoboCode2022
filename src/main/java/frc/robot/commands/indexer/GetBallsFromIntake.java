package frc.robot.commands.indexer;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;

public class GetBallsFromIntake extends IndefiniteCommand {
    public static final double INDEXER_SPEED = 11.5; // meters per second
    public static final double INDEXER_OUTTAKE_SPEED = -16; // meters per second

    public GetBallsFromIntake() {
        addRequirements(Indexer.getInstance());
    }

    public void execute() {
        switch (Intake.getInstance().getState()) {
            case INTAKE:
                if (Indexer.getInstance().isBallInTop()) Indexer.getInstance().setTopOutput(0);
                else Indexer.getInstance().setTopOutput(INDEXER_SPEED);
                if (Indexer.getInstance().isBallInBottom())
                    if (Indexer.getInstance().isBallInTop()) Indexer.getInstance().setBottomOutput(0);
                    else Indexer.getInstance().setBottomOutput(INDEXER_SPEED);
                else Indexer.getInstance().setBottomOutput(INDEXER_SPEED);
                break;
            case OUTTAKE:
                Indexer.getInstance().setBothOutput(INDEXER_OUTTAKE_SPEED);
                break;
            case NEUTRAL:
                Indexer.getInstance().setBothOutput(0);
                break;
        }
    }

    public void end(boolean interrupted) {
        Indexer.getInstance().setBothOutput(0);
    }
}