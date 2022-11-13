package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.Conversions;
import frc.robot.util.HSFalconBuilder;
import frc.robot.util.MotorVelocitySystem;
import harkerrobolib.wrappers.HSFalcon;

public class Intake extends SubsystemBase {
  private static Intake instance;
  private DoubleSolenoid intake;
  private HSFalcon roller;

  private static final double WHEEL_DIAMETER = 2.0;
  public static final double GEAR_RATIO = 5.0 / 3.0;
  public static final double FALCON_VEL_TO_CARGO_SPEED =
      Conversions.ENCODER_TO_WHEEL_SPEED / GEAR_RATIO * (WHEEL_DIAMETER / 2.0);

  private static final double CONTINUOUS_CURRENT_LIMIT = 30;
  private static final double PEAK_CURRENT = 40;
  private static final double PEAK_DUR = 0.1;
  private static final boolean INVERT = true;

  private static final double kS = 0.0836; // tune later
  private static final double kV = 1.1553;
  private static final double kA = 0.147;

  private static final double MAX_ERROR = 0.5;

  private MotorVelocitySystem velocitySystem;

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
    roller =
        new HSFalconBuilder()
            .invert(INVERT)
            .supplyLimit(PEAK_CURRENT, CONTINUOUS_CURRENT_LIMIT, PEAK_DUR)
            .velocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms)
            .build(RobotMap.INTAKE_MOTOR);
    // velocitySystem =
    //     new MotorVelocitySystemBuilder()
    //         .constants(kV, kA, kS)
    //         .maxError(MAX_ERROR)
    //         .unitConversionFactor(FALCON_VEL_TO_CARGO_SPEED)
    //         .build(roller)
    //         .init();
    state = State.NEUTRAL;
    addChild("Motor", roller);
    // addChild("Motor System", velocitySystem);
  }

  public void setForward() {
    intake.set(DoubleSolenoid.Value.kForward);
  }

  public void setBackward() {
    intake.set(DoubleSolenoid.Value.kReverse);
  }

  public void setRollerOutput(double rollerOutput) {
    roller.set(ControlMode.PercentOutput, rollerOutput);
    // velocitySystem.set(rollerOutput);
  }

  // public void turnOffMotor() {
  //   roller.set(ControlMode.PercentOutput, 0);
  // }

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
        setForward();
        break;
      case INTAKE:
        setBackward();
        setRollerOutput(intakeSpeed);
        break;
      case OUTTAKE:
        setBackward();
        setRollerOutput(-intakeSpeed + 0.3);
        break;
    }
  }

  public double getIntakeSpeed() {
    return velocitySystem.getVelocity();
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
    builder.addStringProperty("State", () -> state.name(), (a) -> state = State.valueOf(a));
    builder.addDoubleProperty("Unit Conversion", () -> FALCON_VEL_TO_CARGO_SPEED, null);
  }
}
