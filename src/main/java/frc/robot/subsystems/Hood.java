package frc.robot.subsystems;

import static harkerrobolib.util.Conversions.AngleUnit.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.HSFalconBuilder;
import frc.robot.util.MotorPositionSystem;
import frc.robot.util.MotorPositionSystem.MotorPositionSystemBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class Hood extends SubsystemBase {
  private static Hood instance;

  private HSFalcon hood;

  private DigitalInput limitSwitch;

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
  private static final double FALCON_TO_DEG = TALONFX.to(DEGREE, GEAR_RATIO);

  private static final double MAX_POS_ERROR = 5;
  private static final double MAX_VEL_ERROR = 5;
  private static final double MAX_VOLTAGE = 3;

  private boolean isHoodZeroed;

  private MotorPositionSystem positionSystem;

  private Hood() {
    hood =
        new HSFalconBuilder()
            .statorLimit(CURRENT_PEAK, CURRENT_CONTINUOUS, CURRENT_PEAK_DUR)
            .invert(INVERT)
            .build(RobotMap.HOOD_ID, RobotMap.CANBUS);
    addChild("Hood Motor", hood);
    positionSystem =
        new MotorPositionSystemBuilder()
            .constants(kV, kA, kS)
            .armGravityConstant(kG)
            .unitConversionFactor(FALCON_TO_DEG)
            .maxVoltage(MAX_VOLTAGE)
            .maxError(MAX_POS_ERROR, MAX_VEL_ERROR)
            .build(hood);
    addChild("Hood Position System", positionSystem);
    isHoodZeroed = false;
    initMotors();
  }

  public void initMotors() {
    hood.configForwardSoftLimitEnable(true);
    hood.configForwardSoftLimitThreshold(DEGREE.to(TALONFX, RANGE * GEAR_RATIO));
  }

  public void setHoodPosition(double position) {
    positionSystem.set(position);
  }

  public void setHoodPercentOutput(double percentOutput) {
    hood.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setHoodEncoderZero() {
    hood.setSelectedSensorPosition(0);
    isHoodZeroed = true;
  }

  public double getHoodPosition() {
    return positionSystem.getPosition();
  }

  public double getHoodVelocity() {
    return positionSystem.getVelocity();
  }

  public boolean isLimitSwitchHit() {
    return !limitSwitch.get();
  }

  public boolean isHoodZeroed() {
    return isHoodZeroed;
  }

  public static Hood getInstance() {
    if (instance == null) instance = new Hood();
    return instance;
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Hood");
    builder.addBooleanProperty("Is Zeroed", () -> isHoodZeroed, null);
    builder.addBooleanProperty("Limit Switch Hit", () -> !limitSwitch.get(), null);
  }
}
