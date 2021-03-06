package frc.robot.commands.indexer;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;

public class GetBallsFromIntake extends IndefiniteCommand {
    // public static final double INDEXER_BOTTOM_SPEED_PERCENT_OUTPUT = 0.9;
    // public static final double INDEXER_TOP_SPEED_PERCENT_OUTPUT = 0.35;
    public static final double INDEXER_SPEED = 36;
    public static final double INDEXER_OUTTAKE_SPEED = -50;

    public GetBallsFromIntake() {
        addRequirements(Indexer.getInstance());
    }

    public void execute() {
        switch (Intake.getInstance().getCurrIntakeState()) {
            case INTAKE:
                if (Indexer.getInstance().isBallInBottom() != Indexer.getInstance().isBallInTop()) {
                    if (Indexer.getInstance().isBallInBottom()) {
                        Indexer.getInstance().setBothOutput(INDEXER_SPEED);
                    }
                    else if (Indexer.getInstance().isBallInTop()) {
                        Indexer.getInstance().setTopOutput(0);
                        Indexer.getInstance().setBottomOutput(INDEXER_SPEED);
                    }
                }
                else if (!Indexer.getInstance().isBallInBottom()) { // && !Indexer.getInstance().isBallInTop()
                    Indexer.getInstance().setBothOutput(INDEXER_SPEED);
                }
                else {
                    Indexer.getInstance().setBothOutput(0);
                }
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