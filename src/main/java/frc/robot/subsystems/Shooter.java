package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.HSFalconBuilder;
import frc.robot.util.Units;
import frc.robot.util.loop.VelocityControlLoop;
import frc.robot.util.loop.VelocityControlLoop.VelocityControlLoopBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class Shooter extends SubsystemBase {
  private static Shooter instance;

  private HSFalcon master;
  private HSFalcon follower;

  private VelocityControlLoop velocityLoop;

  private static final boolean MASTER_INVERT = true;
  private static final boolean FOLLOWER_INVERT = false;

  private static final double CURRENT_CONTINUOUS = 40;
  private static final double CURRENT_PEAK = 45;
  private static final double CURRENT_PEAK_DUR = 0.5;

  private static final double kS = 0.1432;
  private static final double kV = 0.51368;
  private static final double kA = 0.038625;
  private static final double MODEL_STDEV = 0.5; // TODO: Tune
  private static final double ENCODER_STDEV = 0.015; // TODO: Tune
  private static final double MAX_ERROR = 1.0; // TODO: Tune

  private static final double VELOCITY_TOLERANCE = 0.1;

  private static final double SHOOTER_GEAR_RATIO = 1.5;

  public static final double CUSTOM_RADIUS = 1.0;

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
    follower =
        new HSFalconBuilder()
            .invert(FOLLOWER_INVERT)
            .neutralMode(NeutralMode.Coast)
            .canFramePeriods(RobotMap.MAX_CAN_FRAME_PERIOD, RobotMap.MAX_CAN_FRAME_PERIOD)
            .build(RobotMap.SHOOTER_FOLLOWER, RobotMap.CANBUS);
    follower.follow(master);
    velocityLoop =
        new VelocityControlLoopBuilder()
            .motorConstants(kS, kA, kV)
            .standardDeviations(MODEL_STDEV, ENCODER_STDEV)
            .maxError(MAX_ERROR)
            .buildVelocityControlLoop();
    state = State.IDLE;
  }

  public void set(double speed) {
    master.setVoltage(velocityLoop.setReferenceAndPredict(speed, getRawShooterSpeed()));
  }

  public double getRawShooterSpeed() {
    return master.getSelectedSensorVelocity()
        * Units.FALCON_VELOCITY_TO_ROT_PER_SECOND
        / SHOOTER_GEAR_RATIO
        * Units.wheelRotsToMeter(4.0);
  }

  public void update() {
    set(velocityLoop.getSetpoint());
  }

  public void turnOffMotors() {
    master.set(ControlMode.PercentOutput, 0);
    follower.set(ControlMode.PercentOutput, 0);
  }

  public double getTargetSpeed() {
    return velocityLoop.getSetpoint();
  }

  public boolean atTargetSpeed(double customTarget) {
    return Math.abs(velocityLoop.getFilteredVelocity() - customTarget) < VELOCITY_TOLERANCE;
  }

  public boolean atTargetSpeed() {
    return Math.abs(getError()) < VELOCITY_TOLERANCE;
  }

  public double getFilteredVelocity() {
    return velocityLoop.getFilteredVelocity();
  }

  public double getError() {
    return velocityLoop.getVelocityError();
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
    builder.addDoubleProperty("Raw Shooter Speed", () -> getRawShooterSpeed(), null);
    builder.addDoubleProperty("Filtered Shooter Speed", () -> getFilteredVelocity(), null);
    builder.addDoubleProperty("Shooter Voltage", () -> master.getMotorOutputVoltage(), null);
  }
}
