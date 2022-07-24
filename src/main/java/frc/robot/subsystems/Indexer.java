package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.ColorSensor;
import frc.robot.util.HSFalconBuilder;
import frc.robot.util.Units;
import frc.robot.util.loop.VelocityControlLoop;
import frc.robot.util.loop.VelocityControlLoop.VelocityControlLoopBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class Indexer extends SubsystemBase {
  private static Indexer instance;
  private HSFalcon top;
  private HSFalcon bottom;
  private DigitalInput topProximity;
  private DigitalInput bottomProximity;
  private ColorSensor colorSensor;

  private static final double CURRENT_CONTINUOUS = 30;
  private static final double CURRENT_PEAK = 60;
  private static final double CURRENT_PEAK_DUR = 0.2;
  private static final boolean TOP_INVERT = true;
  private static final boolean BOTTOM_INVERT = false;

  private static final double INDEXER_TOP_GEAR_RATIO = 5.0;
  private static final double INDEXER_BOTTOM_GEAR_RATIO = 1.5;

  private static final double TOP_kS = 0.14855; // TODO: Tune
  private static final double TOP_kV = 2.5874; // TODO: Tune
  private static final double TOP_kA = 0.049339; // TODO: Tune
  private static final double BOTTOM_kS = 0.050444; // TODO: Tune
  private static final double BOTTOM_kV = 1.704; // TODO: Tune
  private static final double BOTTOM_kA = 0.026034; // TODO: Tune

  private static final double MAX_ERROR = 0.5; // TODO: Tune
  private static final double TOP_MODEL_STDEV = 0.5; // TODO: Tune
  private static final double TOP_ENCODER_STDEV = 0.035; // TODO: Tune

  private static final double BOTTOM_MODEL_STDEV = 0.5; // TODO: Tune
  private static final double BOTTOM_ENCODER_STDEV = 0.035; // TODO: Tune

  private VelocityControlLoop bottomVelocityLoop;
  private VelocityControlLoop topVelocityLoop;

  private Indexer() {
    top =
        new HSFalconBuilder()
            .invert(TOP_INVERT)
            .statorLimit(CURRENT_PEAK, CURRENT_CONTINUOUS, CURRENT_PEAK_DUR)
            .build(RobotMap.INDEXER_TOP, RobotMap.CANBUS);
    bottom =
        new HSFalconBuilder()
            .invert(BOTTOM_INVERT)
            .statorLimit(CURRENT_PEAK, CURRENT_CONTINUOUS, CURRENT_PEAK_DUR)
            .build(RobotMap.INDEXER_BOTTOM, RobotMap.CANBUS);
    topProximity = new DigitalInput(RobotMap.TOP_PROXIMITY);
    bottomProximity = new DigitalInput(RobotMap.BOTTOM_PROXIMITY);
    topVelocityLoop =
        new VelocityControlLoopBuilder()
            .motorConstants(TOP_kS, TOP_kA, TOP_kV)
            .standardDeviations(TOP_MODEL_STDEV, TOP_ENCODER_STDEV)
            .maxError(MAX_ERROR)
            .buildVelocityControlLoop();
    bottomVelocityLoop =
        new VelocityControlLoopBuilder()
            .motorConstants(BOTTOM_kS, BOTTOM_kA, BOTTOM_kV)
            .standardDeviations(BOTTOM_MODEL_STDEV, BOTTOM_ENCODER_STDEV)
            .maxError(MAX_ERROR)
            .buildVelocityControlLoop();
    colorSensor = new ColorSensor(RobotMap.COLOR_A, RobotMap.COLOR_B, RobotMap.COLOR_PROXIMITY);
  }

  public void setTopOutput(double topOutput) {
    top.setVoltage(topVelocityLoop.setReferenceAndPredict(topOutput, getTopMPS()));
  }

  public void setBottomOutput(double bottomOutput) {
    bottom.setVoltage(bottomVelocityLoop.setReferenceAndPredict(bottomOutput, getBottomMPS()));
  }

  public void setBothOutput(double output) {
    setTopOutput(output);
    setBottomOutput(output);
  }

  public void turnOffMotors() {
    top.set(ControlMode.PercentOutput, 0);
    bottom.set(ControlMode.PercentOutput, 0);
  }

  public boolean isBallInTop() {
    return !topProximity.get();
  }

  public boolean isBallInBottom() {
    return !bottomProximity.get();
  }

  public double getTopMPS() {
    return Units.wheelRotsToMeter(4.0)
        * top.getSelectedSensorVelocity()
        * Units.FALCON_VELOCITY_TO_ROT_PER_SECOND
        / INDEXER_TOP_GEAR_RATIO; // change
  }

  public double getBottomMPS() {
    return Units.wheelRotsToMeter(3.0)
        * bottom.getSelectedSensorVelocity()
        * Units.FALCON_VELOCITY_TO_ROT_PER_SECOND
        / INDEXER_BOTTOM_GEAR_RATIO; // change
  }

  public static Indexer getInstance() {
    if (instance == null) {
      instance = new Indexer();
    }
    return instance;
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Indexer");
    builder.addDoubleProperty("Top Indexer Velocity", () -> getTopMPS(), null);
    builder.addDoubleProperty("Bottom Indexer Velocity", () -> getBottomMPS(), null);
    builder.addBooleanProperty("Color is red", () -> colorSensor.isRed(), null);
    builder.addBooleanProperty(
        "Color sensor is functioning", () -> colorSensor.isFunctioning(), null);
  }
}
