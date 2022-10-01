package frc.robot.commands.hood;

import frc.robot.subsystems.Hood;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.PhotonVisionLimelight;
import harkerrobolib.commands.IndefiniteCommand;

public class HoodManual extends IndefiniteCommand {
  private InterpolatingTreeMap hoodVals;

  public HoodManual() {
    addRequirements(Hood.getInstance());
    hoodVals = new InterpolatingTreeMap();
    hoodVals.put(1.15, 0.0);
    hoodVals.put(2.8, 23.0);
    hoodVals.put(3.2, 24.5);
    hoodVals.put(3.4, 25.5);
    hoodVals.put(3.7, 26.0);
    hoodVals.put(3.9, 27.0);
    hoodVals.put(4.19, 28.7);
    hoodVals.put(4.57, 29.0);
    hoodVals.put(4.85, 30.0);
    hoodVals.put(5.2, 32.0);
    hoodVals.put(5.4, 32.0);
    hoodVals.put(5.78, 32.0);
    hoodVals.put(6.26, 32.0);
    hoodVals.put(3.2, 11.0);
  }

  public void execute() {
    Hood.getInstance().setHoodPosition(calculateHoodPosition());
  }

  public double calculateHoodPosition() {
    // return SmartDashboard.getNumber("angle", 0.0);
    return hoodVals.get(PhotonVisionLimelight.getDistance());
  }

  public void end(boolean interrupted) {
    Hood.getInstance().setHoodPosition(0);
  }
}
