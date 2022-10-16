package frc.robot.commands.hood;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Hood;
import harkerrobolib.commands.IndefiniteCommand;

public class HoodManual extends IndefiniteCommand {

  public HoodManual() {
    addRequirements(Hood.getInstance());
  }

  public void execute() {
    SmartDashboard.putBoolean("yeetmcfeetus", true);
    SmartDashboard.putNumber("hood pos calculate", Hood.getInstance().calculateHoodPosition());
    Hood.getInstance().setHoodPosition(Hood.getInstance().calculateHoodPosition());
  }

  public void end(boolean interrupted) {
    Hood.getInstance().setHoodPosition(0);
  }
}
