package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class MotorPositionSystem extends MotorVelocitySystem {
  private double kG;
  public boolean arm;
  private double positionSetpoint;
  private double maxPosError;

  private MotorPositionSystem(
      BaseMotorController motor,
      double kD,
      double kP,
      double kS,
      double unitConversion,
      double maxPosError,
      double maxVelError,
      double kG,
      boolean arm) {
    super(motor, kS, 0.0, unitConversion, kP, maxVelError);
    motor.config_kD(RobotMap.SLOT_INDEX, kD);
    this.kG = kG;
    this.arm = arm;
    this.maxPosError = maxPosError;
  }

  public void set(double position) {
    positionSetpoint = position;
    velocitySetpoint = 0.0;
    SmartDashboard.putBoolean("functioning", true);
    double ff = Math.signum(getPositionError()) * kS;
    if (arm) ff += kG * Math.cos(Math.toRadians(getPosition()));
    else ff += kG;
    motor.set(ControlMode.Position, position / unitConversion, DemandType.ArbitraryFeedForward, ff);
  }

  public double getVelocitySetpoint() {
    return 0.0;
  }

  public double getVelocity() {
    return super.getVelocity() * Conversions.SECOND_TO_CTRE_SECOND;
  }

  public double getPosition() {
    return motor.getSelectedSensorPosition() * unitConversion;
  }

  public double getPositionError() {
    return positionSetpoint - getPosition();
  }

  public double getPositionSetpoint() {
    return positionSetpoint;
  }

  public boolean atSetpoint() {
    return super.atSetpoint() && Math.abs(getPositionError()) < maxPosError;
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("MotorPositionSystem");
    builder.addDoubleProperty("Velocity", () -> this.getVelocity(), null);
    builder.addDoubleProperty("Position", () -> getPosition(), (a) -> set(a));
  }

  public static class MotorPositionSystemBuilder {
    private double kV,
        kA,
        kS,
        maxPosError,
        unitConversion = 1.0,
        maxVoltage = RobotMap.MAX_MOTOR_VOLTAGE,
        maxVelError,
        kG = 0.0;
    private boolean arm;

    public MotorPositionSystemBuilder maxError(double posError, double velError) {
      this.maxPosError = posError;
      this.maxVelError = velError;
      return this;
    }

    public MotorPositionSystemBuilder elevatorGravityConstant(double kG) {
      this.kG = kG;
      this.arm = false;
      return this;
    }

    public MotorPositionSystemBuilder armGravityConstant(double kG) {
      this.kG = kG;
      this.arm = true;
      return this;
    }

    public MotorPositionSystemBuilder constants(double kV, double kA, double kS) {
      this.kV = kV;
      this.kA = kA;
      this.kS = kS;
      return this;
    }

    public MotorPositionSystemBuilder unitConversionFactor(double factor) {
      unitConversion = factor;
      return this;
    }

    public MotorPositionSystemBuilder maxVoltage(double voltage) {
      maxVoltage = voltage;
      return this;
    }

    public MotorPositionSystem build(BaseMotorController motor) {
      Matrix<N1, N2> gains =
          new LinearQuadraticRegulator<>(
                  LinearSystemId.identifyPositionSystem(kV, kA),
                  VecBuilder.fill(maxPosError, maxVelError),
                  VecBuilder.fill(maxVoltage),
                  RobotMap.TALON_FX_LOOP)
              .getK();
      gains.times(1023.0 / maxVoltage * unitConversion);
      kS /= maxVoltage;
      return new MotorPositionSystem(
          motor,
          gains.get(0, 1),
          gains.get(0, 0),
          kS,
          unitConversion,
          maxPosError,
          maxVelError,
          kG,
          arm);
    }
  }
}
