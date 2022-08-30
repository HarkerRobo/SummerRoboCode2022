package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.HSFalconBuilder;
import frc.robot.util.MotorPositionSystem;
import frc.robot.util.MotorPositionSystem.MotorPositionSystemBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class Climber extends SubsystemBase {
  private static Climber instance;

  private HSFalcon left;
  private HSFalcon right;
  private DoubleSolenoid climber;
  private DigitalInput rightLimitSwitch;
  private DigitalInput leftLimitSwitch;

  private static final double CURRENT_CONTINUOUS = 40;
  private static final double CURRENT_PEAK = 45;
  private static final double CURRENT_PEAK_DUR = 0.5;

  private static final double RIGHT_kS = 0;
  private static final double RIGHT_kV = 0;
  private static final double RIGHT_kA = 0;
  private static final double RIGHT_kG = 0;
  private static final boolean RIGHT_INVERT = false;

  private static final double LEFT_kS = 0;
  private static final double LEFT_kV = 0;
  private static final double LEFT_kA = 0;
  private static final double LEFT_kG = 0;
  private static final double POS_MAX_ERROR = 1;
  private static final double VEL_MAX_ERROR = 1;
  private static final boolean LEFT_INVERT = true;

  public static final double UP_HEIGHT = 119000; // TODO
  public static final double MID_HEIGHT = 60000; // TODO
  public static final double DOWN_HEIGHT = 0; // TODO

  private MotorPositionSystem leftPositionSys;
  private MotorPositionSystem rightPositionSys;

  private Climber() {
    right =
        new HSFalconBuilder()
            .invert(RIGHT_INVERT)
            .statorLimit(CURRENT_PEAK, CURRENT_CONTINUOUS, CURRENT_PEAK_DUR)
            .build(RobotMap.RIGHT_CLIMBER, RobotMap.CANBUS);
    addChild("Right Motor", right);
    left =
        new HSFalconBuilder()
            .invert(LEFT_INVERT)
            .statorLimit(CURRENT_PEAK, CURRENT_CONTINUOUS, CURRENT_PEAK_DUR)
            .build(RobotMap.LEFT_CLIMBER, RobotMap.CANBUS);
    addChild("Left Motor", left);
    climber =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH, RobotMap.CLIMBER_FORWARD, RobotMap.CLIMBER_BACKWARD);
    rightLimitSwitch = new DigitalInput(RobotMap.CLIMBER_RIGHT_LIMIT_SWTICH);
    leftLimitSwitch = new DigitalInput(RobotMap.CLIMBER_LEFT_LIMIT_SWITCH);
    rightPositionSys =
        new MotorPositionSystemBuilder()
            .maxError(POS_MAX_ERROR, VEL_MAX_ERROR)
            .elevatorGravityConstant(RIGHT_kG)
            .constants(RIGHT_kV, RIGHT_kA, RIGHT_kS)
            .build(right);
    leftPositionSys =
        new MotorPositionSystemBuilder()
            .maxError(POS_MAX_ERROR, VEL_MAX_ERROR)
            .elevatorGravityConstant(LEFT_kG)
            .constants(LEFT_kV, LEFT_kA, LEFT_kS)
            .build(right);
    addChild("Right Position System", rightPositionSys);
    addChild("Left Position System", leftPositionSys);
  }

  public void setRightClimberPos(double pos) {
    rightPositionSys.set(pos);
  }

  public void setLeftClimberPos(double pos) {
    leftPositionSys.set(pos);
  }

  public boolean limitSwitchHit() {
    return !rightLimitSwitch.get() && !leftLimitSwitch.get();
  }

  public void setBothClimberPos(double pos) {
    setRightClimberPos(pos);
    setLeftClimberPos(pos);
  }

  public void setClimberForward() {
    climber.set(DoubleSolenoid.Value.kReverse);
  }

  public void setClimberBackward() {
    climber.set(DoubleSolenoid.Value.kForward);
  }

  public void turnOffMotors() {
    right.set(ControlMode.PercentOutput, 0);
    left.set(ControlMode.PercentOutput, 0);
  }

  public double getRightClimberPos() {
    return right.getSelectedSensorPosition();
  }

  public double getLeftClimberPos() {
    return left.getSelectedSensorPosition();
  }

  public double getRightClimberVel() {
    return right.getSelectedSensorVelocity();
  }

  public double getLeftClimberVel() {
    return left.getSelectedSensorVelocity();
  }

  public MotorPositionSystem getRightPositionSystem() {
    return rightPositionSys;
  }

  public MotorPositionSystem getLeftPositionSystem() {
    return leftPositionSys;
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Climber");
    builder.addStringProperty("Solenoid Value", () -> climber.get().name(), null);
    builder.addBooleanProperty("Left Limit Switch Hit", () -> !leftLimitSwitch.get(), null);
    builder.addBooleanProperty("Right Limit Switch Hit", () -> !rightLimitSwitch.get(), null);
  }

  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }
}
