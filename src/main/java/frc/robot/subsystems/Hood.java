package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.HSFalconBuilder;
import frc.robot.util.Units;
import frc.robot.util.loop.PositionControlLoop;
import frc.robot.util.loop.PositionControlLoop.PositionControlLoopBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class Hood extends SubsystemBase {
  private static Hood instance;

  private HSFalcon hood;

  private static final boolean INVERT = false;

  private static final double CURRENT_CONTINUOUS = 10;
  private static final double CURRENT_PEAK = 10;
  private static final double CURRENT_PEAK_DUR = 0.05;
  private static final int RANGE = 23;

  private static final double kS = 0;
  private static final double kV = 0;
  private static final double kA = 0;
  private static final double kG = 0.087132;

  private static final double GEAR_RATIO = 180; // needs to be updated

  private static final double MAX_POS_ERROR = 5;
  private static final double MAX_VEL_ERROR = 5;
  private static final double MODEL_POS_STDEV = 0.5;
  private static final double MODEL_VEL_STDEV = 0.5;
  private static final double ENCODER_STDEV = 0.035;
  private static final double MAX_VOLTAGE = 3;

  private static final double STALLING_CURRENT = 10;

  private boolean isHoodZeroed;

  private PositionControlLoop positionLoop;

  private Hood() {
    hood =
        new HSFalconBuilder()
            .statorLimit(CURRENT_PEAK, CURRENT_CONTINUOUS, CURRENT_PEAK_DUR)
            .invert(INVERT)
            .build(RobotMap.HOOD_ID, RobotMap.CANBUS);
    positionLoop =
        new PositionControlLoopBuilder()
            .motorConstants(kS, kA, kV, kG)
            .standardDeviations(MODEL_POS_STDEV, MODEL_VEL_STDEV, ENCODER_STDEV)
            .maxError(MAX_POS_ERROR, MAX_VEL_ERROR)
            .maxControlEffort(MAX_VOLTAGE)
            .buildArmControlLoop();
    isHoodZeroed = false;
    initMotors();
  }

  public void initMotors() {
    hood.configForwardSoftLimitEnable(true);
    hood.configForwardSoftLimitThreshold(RANGE * GEAR_RATIO * Units.DEGREES_TO_ENCODER_TICKS);
  }

  public void setHoodPosition(double position) {
    hood.setVoltage(positionLoop.setReferenceAndPredict(position, 0.0, getHoodPosition()));
  }

  public void setHoodPercentOutput(double percentOutput) {
    hood.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setHoodEncoderZero() {
    hood.setSelectedSensorPosition(0);
    isHoodZeroed = true;
  }

  public double getHoodPosition() {
    return hood.getSelectedSensorPosition() * Units.ENCODER_TICKS_TO_DEGREES / GEAR_RATIO;
  }

  public double getHoodVelocity() {
    return hood.getSelectedSensorVelocity() * Units.ENCODER_TICKS_TO_DEGREES / GEAR_RATIO;
  }

  public boolean isHoodStalling() {
    return hood.getStatorCurrent() >= STALLING_CURRENT;
  }

  public boolean isHoodZeroed() {
    return isHoodZeroed;
  }

  public static Hood getInstance() {
    if (instance == null) instance = new Hood();
    return instance;
  }
}
