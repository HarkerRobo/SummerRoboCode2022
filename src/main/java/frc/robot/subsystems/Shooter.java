package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.Conversions;
import frc.robot.util.HSFalconBuilder;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.PhotonVisionLimelight;
import harkerrobolib.wrappers.HSFalcon;

public class Shooter extends SubsystemBase {
  private static Shooter instance;

  private HSFalcon master;
  private HSFalcon follower;

  private InterpolatingTreeMap shooterVal;

  private static final boolean MASTER_INVERT = false;
  private static final boolean FOLLOWER_INVERT = true;

  private static final double CURRENT_CONTINUOUS = 40;
  private static final double CURRENT_PEAK = 100;
  private static final double CURRENT_PEAK_DUR = 0.5;

  private static final double kS = 0.66; // .27;
  private static final double kV = 0.64;
  private static final double kA = 0.06045;

  private static final double kP = 0.17;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static final double SHOOT_ERROR = 0.07;

  private static final double WHEEL_DIAMETER = 4.0;

  private static final double SHOOTER_GEAR_RATIO = 1.5;

  private static final double MOTOR_TO_METERS_PER_SECOND =
      Conversions.ENCODER_TO_WHEEL_SPEED * WHEEL_DIAMETER / SHOOTER_GEAR_RATIO;

  // private MotorVelocitySystem velocitySystem;

  private static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(kS, kV, kA);
  private static final PIDController PID = new PIDController(kP, kI, kD);

  private State state;

  private Debouncer speedDebounce;

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
            .velocityWindow(8)
            .velocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms)
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
    shooterVal = insertShooterVals();
    // velocitySystem =
    //     new MotorVelocitySystemBuilder()
    //         .constants(kV, kA, kS)
    //         .unitConversionFactor(MOTOR_TO_METERS_PER_SECOND)
    //         .maxError(MAX_ERROR)
    //         .build(master)
    //         .init();
    // addChild("Velocity System", velocitySystem);
    state = State.IDLE;
    master.configOpenloopRamp(0.7);
    speedDebounce = new Debouncer(0.01, DebounceType.kRising);
  }

  private InterpolatingTreeMap insertShooterVals() {
    InterpolatingTreeMap shooterVals = new InterpolatingTreeMap();
    shooterVals.put(2.3, 13.4);
    shooterVals.put(2.6, 13.6);
    shooterVals.put(2.8, 13.8);
    shooterVals.put(3.1, 14.0);
    shooterVals.put(3.5, 14.1);
    shooterVals.put(3.8, 14.4);
    shooterVals.put(4.1, 14.8);
    shooterVals.put(4.38, 15.0);
    shooterVals.put(4.62, 15.3);
    shooterVals.put(5.0, 15.5);
    shooterVals.put(5.3, 16.2);
    shooterVals.put(5.6, 16.3);
    shooterVals.put(5.83, 16.5);
    return shooterVals;
  }

  public void set(double speed) {
    master.setVoltage(FEEDFORWARD.calculate(speed) + PID.calculate(getSpeed(), speed));
    // velocitySystem.set(speed);
  }

  public void resetPID() {
    PID.reset();
  }

  public double getSpeed() {
    return master.getSelectedSensorVelocity() * MOTOR_TO_METERS_PER_SECOND;
  }

  public void turnOffMotors() {
    master.set(ControlMode.PercentOutput, 0);
  }

  public boolean atSpeed(double speed) {
    return speedDebounce.calculate(Math.abs(speed - PID.getSetpoint()) < SHOOT_ERROR);
  }

  public double calculateShooterSpeed() {
    return shooterVal.get(PhotonVisionLimelight.getDistance());
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
    builder.addDoubleProperty("velocity", () -> getSpeed(), null);
    builder.addDoubleProperty("desired velocity", () -> PID.getSetpoint(), null);
  }
}
