package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.ClimberStages;
import frc.robot.commands.climber.SetClimberPos;
import frc.robot.commands.climber.ZeroClimber;
import frc.robot.commands.hood.ZeroHood;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
// import frc.robot.commands.climber.ClimberStages;
import harkerrobolib.joysticks.XboxGamepad;

public class OI {
  private static OI instance;
  private XboxGamepad operator;
  private XboxGamepad driver;

  public static final double DEFAULT_DEADBAND = 0.15;

  private OI() {
    operator = new XboxGamepad(RobotMap.OPERATOR_ID);
    driver = new XboxGamepad(RobotMap.DRIVER_ID);
    initBindings();
  }

  public void initBindings() {
    driver.getButtonA().whenPressed(ClimberStages.ALL_STAGES);
    driver
        .getButtonStart()
        .whenPressed(
            new InstantCommand(
                () -> {
                  Drivetrain.getInstance().setPose(new Pose2d());
                  Drivetrain.getInstance().zeroPigeon();
                }));
    driver.getButtonStart().whenPressed(new ZeroClimber());
    driver.getButtonX().whenPressed(new ZeroHood());
    driver.getButtonY().whenPressed(()->Indexer.getInstance().setBothOutput(18));
    // driver.getButtonB().whenPressed(new SetClimberPos(Climber.UP_HEIGHT));
    // driver.getButtonX().whenPressed(new SetClimberPos(Climber.MID_HEIGHT));
    driver.getUpDPadButton().whenPressed(()->{Climber.getInstance().setRightPercentOutput(0.1); Climber.getInstance().setLeftPercentOutput(0.1);});
    driver.getDownDPadButton().whenPressed(()->{Climber.getInstance().setRightPercentOutput(-0.1); Climber.getInstance().setLeftPercentOutput(-0.1);});
  }

  public XboxGamepad getDriverGamepad() {
    return driver;
  }

  public XboxGamepad getOperatorGamepad() {
    return operator;
  }

  public static OI getInstance() {
    if (instance == null) {
      instance = new OI();
    }
    return instance;
  }
}
