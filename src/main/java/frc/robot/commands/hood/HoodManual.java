package frc.robot.commands.hood;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Hood;
import frc.robot.util.InterpolatingTreeMap;
import harkerrobolib.commands.IndefiniteCommand;

public class HoodManual extends IndefiniteCommand {
  private InterpolatingTreeMap hoodVals;

  public HoodManual() {
    addRequirements(Hood.getInstance());
    hoodVals = new InterpolatingTreeMap();
  }

  public void execute() {
    Hood.getInstance().setHoodPosition(calculateHoodPosition());
  }

  public double calculateHoodPosition() {
    return SmartDashboard.getNumber("angle", 0.0);
    // return hoodVals.get(
    //     Drivetrain.getInstance()
    //         .getPoseEstimator()
    //         .getEstimatedPosition()
    //         .getTranslation()
    //         .getDistance(Constants.HUB_LOCATION));
  }

  public void end(boolean interrupted) {
    Hood.getInstance().setHoodPosition(0);
  }
}
