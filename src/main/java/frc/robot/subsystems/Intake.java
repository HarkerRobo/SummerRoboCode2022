package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.HSFalconBuilder;
import frc.robot.util.Units;
import frc.robot.util.loop.VelocityControlLoop;
import frc.robot.util.loop.VelocityControlLoop.VelocityControlLoopBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class Intake extends SubsystemBase {
  private static Intake instance;
  private DoubleSolenoid intake;
  private HSFalcon roller;

  public static final double INTAKE_GEAR_RATIO = 5.0 / 3.0;

  private static final double CONTINUOUS_CURRENT_LIMIT = 60;
  private static final double PEAK_CURRENT = 60;
  private static final double PEAK_DUR = 0;
  private static final boolean INVERT = true;

  private static final double kS = 0.0836; // tune later
  private static final double kV = 1.1553;
  private static final double kA = 0.147;

  private static final double MAX_ERROR = 0.01;
  private static final double MODEL_STDEV = 0.5;
  private static final double ENCODER_STDEV = 0.015;

  private VelocityControlLoop loop;

  private State state;

  public static enum State {
    INTAKE,
    NEUTRAL,
    OUTTAKE
  }

  private Intake() {
    intake =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH, RobotMap.INTAKE_FORWARD, RobotMap.INTAKE_BACKWARD);
    roller = new HSFalconBuilder()
              .invert(INVERT)
              .supplyLimit(PEAK_CURRENT, CONTINUOUS_CURRENT_LIMIT, PEAK_DUR)
              .velocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms)
              .build(RobotMap.INTAKE_MOTOR, "rio");
    loop =
        new VelocityControlLoopBuilder()
            .motorConstants(kS, kA, kV)
            .standardDeviations(MODEL_STDEV, ENCODER_STDEV)
            .maxError(MAX_ERROR)
            .buildVelocityControlLoop();
    state = State.NEUTRAL;
  }

  public void setForward() {
    intake.set(DoubleSolenoid.Value.kForward);
  }

  public void setBackward() {
    intake.set(DoubleSolenoid.Value.kReverse);
  }

  public void setRollerOutput(double rollerOutput) {
    roller.setVoltage(loop.setReferenceAndPredict(rollerOutput, getIntakeSpeed()));
  }

  public void turnOffMotor() {
    roller.set(ControlMode.PercentOutput, 0);
  }

  public State getState() {
    return state;
  }

  public void setState(State s) {
    state = s;
  }

  public void actOnState(double intakeSpeed) {
    switch (Intake.getInstance().getState()) {
      case NEUTRAL:
        setRollerOutput(0);
        roller.set(ControlMode.PercentOutput, 0);
        setForward();
        break;
      case INTAKE:
        setRollerOutput(intakeSpeed);
        setBackward();
        break;
      case OUTTAKE:
        setBackward();
        setRollerOutput(-intakeSpeed);
        break;
    }
  }

  public double getIntakeSpeed() {
    return roller.getSelectedSensorVelocity()
        / INTAKE_GEAR_RATIO
        * Units.FALCON_VELOCITY_TO_ROT_PER_SECOND
        * Units.wheelRotsToMeter(2);
  }

  public HSFalcon getRollerMotor() {
    return roller;
  }

  public static Intake getInstance() {
    if (instance == null) instance = new Intake();
    return instance;
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake");
    builder.addDoubleProperty(
        "Raw Roller Velocity", () -> getIntakeSpeed(), null);
    builder.addDoubleProperty("Roller Speed", () -> loop.getFilteredVelocity(), null);
    builder.addDoubleProperty("Voltage", () -> roller.getBusVoltage() , null);
    builder.addDoubleProperty("Roller Loop Reference", () -> loop.getSetpoint(), null);
    builder.addDoubleProperty("Roller Loop Error", () -> loop.getVelocityError(), null);
    builder.addDoubleProperty("Roller Loop Feedforward output", () -> loop.getFeedforward().getUff(0), null);
    builder.addDoubleProperty("Roller Loop Output", () -> loop.getController().getU().get(0,0), null);
    builder.addDoubleProperty("Roller Stator Current", ()->roller.getStatorCurrent(), null);
    builder.addDoubleProperty("Roller Supply Current", ()->roller.getSupplyCurrent(), null);

  }
}
