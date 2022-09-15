package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.Conversions;
import frc.robot.util.HSFalconBuilder;
import frc.robot.util.MotorVelocitySystem;
import frc.robot.util.MotorVelocitySystem.MotorVelocitySystemBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class Shooter extends SubsystemBase {
  private static Shooter instance;

  private HSFalcon master;
  private HSFalcon follower;

  private static final boolean MASTER_INVERT = true;
  private static final boolean FOLLOWER_INVERT = false;

  private static final double CURRENT_CONTINUOUS = 40;
  private static final double CURRENT_PEAK = 45;
  private static final double CURRENT_PEAK_DUR = 0.5;

  private static final double kS = 0.26437;
  private static final double kV = 0.58046;
  private static final double kA = 1.0667;
  private static final double MAX_ERROR = 0.1; // TODO: Tune

  private static final double WHEEL_DIAMETER = 4.0;

  private static final double SHOOTER_GEAR_RATIO = 1.5;

  private static final double MOTOR_TO_METERS_PER_SECOND =
      Conversions.ENCODER_TO_WHEEL_SPEED * WHEEL_DIAMETER / SHOOTER_GEAR_RATIO;

  public static final double CUSTOM_RADIUS = 1.0;

  private MotorVelocitySystem velocitySystem;

  private State state;

  public static enum State {
    IDLE,
    REVVING,
    SHOOTING
  }

  private Shooter() {
    master =
        new HSFalconBuilder()
            .invert(MASTER_INVERT)
            .neutralMode(NeutralMode.Coast)
            .supplyLimit(CURRENT_PEAK, CURRENT_CONTINUOUS, CURRENT_PEAK_DUR)
            .build(RobotMap.SHOOTER_MASTER, RobotMap.CANBUS);
    addChild("Master Motor", master);
    follower =
        new HSFalconBuilder()
            .invert(FOLLOWER_INVERT)
            .neutralMode(NeutralMode.Coast)
            .canFramePeriods(RobotMap.MAX_CAN_FRAME_PERIOD, RobotMap.MAX_CAN_FRAME_PERIOD)
            .build(RobotMap.SHOOTER_FOLLOWER, RobotMap.CANBUS);
    follower.follow(master);
    addChild("Follower Motor", follower);
    velocitySystem =
        new MotorVelocitySystemBuilder()
            .constants(kV, kA, kS)
            .unitConversionFactor(MOTOR_TO_METERS_PER_SECOND)
            .maxError(MAX_ERROR)
            .build(master).init();
    addChild("Velocity System", velocitySystem);
    state = State.IDLE;
  }

  public void set(double speed) {
    velocitySystem.set(speed);
  }

  public double getSpeed() {
    return velocitySystem.getVelocity();
  }

  public void turnOffMotors() {
    master.set(ControlMode.PercentOutput, 0);
  }

  public boolean atSpeed(double speed) {
    return Math.abs(master.getSelectedSensorVelocity() * MOTOR_TO_METERS_PER_SECOND - speed)
        < MAX_ERROR;
  }

  public void setState(State nextState) {
    state = nextState;
  }

  public State getState() {
    return state;
  }

  public static Shooter getInstance() {
    if (instance == null) instance = new Shooter();
    return instance;
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter");
    builder.addStringProperty("State", () -> state.name(), (a) -> state = State.valueOf(a));
    builder.addDoubleProperty("Unit Conversion", () -> MOTOR_TO_METERS_PER_SECOND, null);
  }
}
