package frc.robot.commands.indexer;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;

public class GetBallsFromIntake extends IndefiniteCommand {
    // public static final double INDEXER_BOTTOM_SPEED_PERCENT_OUTPUT = 0.9;
    // public static final double INDEXER_TOP_SPEED_PERCENT_OUTPUT = 0.35;
    public static final double INDEXER_SPEED_PERCENT_OUTPUT = 0.5;
    public static final double INDEXER_OUTTAKE_SPEED_PERCENT_OUTPUT = -0.7;

    public GetBallsFromIntake() {
        addRequirements(Indexer.getInstance());
    }

    public void execute() {
        switch (Intake.getInstance().getCurrIntakeState()) {
            case INTAKE:
                if (Indexer.getInstance().isBallInBottom() != Indexer.getInstance().isBallInTop()) {
                    if (Indexer.getInstance().isBallInBottom()) {
                        Indexer.getInstance().setBothOutput(INDEXER_SPEED_PERCENT_OUTPUT);
                    }
                    else if (Indexer.getInstance().isBallInTop()) {
                        Indexer.getInstance().setBottomOutput(INDEXER_SPEED_PERCENT_OUTPUT);
                    }
                }
                else if (!Indexer.getInstance().isBallInBottom()) { // && !Indexer.getInstance().isBallInTop()
                    Indexer.getInstance().setBothOutput(INDEXER_SPEED_PERCENT_OUTPUT);
                }
                else {
                    Indexer.getInstance().setBothOutput(0);
                }
                break;
            case OUTTAKE:
                Indexer.getInstance().setBothOutput(INDEXER_OUTTAKE_SPEED_PERCENT_OUTPUT);
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