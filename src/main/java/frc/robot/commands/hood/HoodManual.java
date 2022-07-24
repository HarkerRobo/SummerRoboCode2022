package frc.robot.commands.hood;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.util.FieldConstants;
import frc.robot.util.InterpolatingTreeMap;
import harkerrobolib.commands.IndefiniteCommand;

public class HoodManual extends IndefiniteCommand{
    private InterpolatingTreeMap hoodVals;

    public HoodManual() {
        addRequirements(Hood.getInstance());
        hoodVals = new InterpolatingTreeMap();
    }

    public void execute() {
        Hood.getInstance().setHoodPosition(calculateHoodPosition());
    }

    public double calculateHoodPosition() {
        return hoodVals.get(Drivetrain.getInstance().getPoseEstimator()
            .getEstimatedPosition().getTranslation().getDistance(FieldConstants.HUB_LOCATION));
    }

    public void end(boolean interrupted) {
        Hood.getInstance().setHoodPosition(0);
    }
    
}
