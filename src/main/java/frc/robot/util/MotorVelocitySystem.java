package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class MotorVelocitySystem implements Sendable {
  protected BaseMotorController motor;
  protected double kS;
  protected double unitConversion;
  protected double velocitySetpoint;
  protected double maxError;

  protected MotorVelocitySystem(
      BaseMotorController motor,
      double kS,
      double kV,
      double unitConversion,
      double kP,
      double maxError) {
    this.motor = motor;
    this.kS = kS;
    this.unitConversion = unitConversion;
    this.maxError = maxError;
    motor.config_kP(RobotMap.SLOT_INDEX, kP);
    motor.config_kF(RobotMap.SLOT_INDEX, kV);
    motor.selectProfileSlot(RobotMap.SLOT_INDEX, 0);
  }

  public void set(double output) {
    motor.set(
        ControlMode.Velocity,
        output / unitConversion,
        DemandType.ArbitraryFeedForward,
        kS * Math.signum(output));
  }

  public double getVelocity() {
    return motor.getSelectedSensorVelocity() * unitConversion;
  }

  public double getVelocityError() {
    return velocitySetpoint - getVelocity();
  }

  public double getVelocitySetpoint() {
    return velocitySetpoint;
  }

  public boolean atSetpoint() {
    return Math.abs(getVelocityError()) < maxError;
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("MotorVelocitySystem");
    builder.addDoubleProperty("Velocity", () -> getVelocity(), (a) -> set(a));
  }

  public static class MotorVelocitySystemBuilder {
    private double kV,
        kA,
        kS,
        maxError,
        unitConversion = 1.0,
        maxVoltage = RobotMap.MAX_MOTOR_VOLTAGE;

    public MotorVelocitySystemBuilder constants(double kV, double kA, double kS) {
      this.kV = kV;
      this.kA = kA;
      this.kS = kS;
      return this;
    }

    public MotorVelocitySystemBuilder unitConversionFactor(double factor) {
      unitConversion = factor;
      return this;
    }

    public MotorVelocitySystemBuilder maxError(double error) {
      maxError = error;
      return this;
    }

    public MotorVelocitySystemBuilder maxVoltage(double voltage) {
      maxVoltage = voltage;
      return this;
    }

    public MotorVelocitySystem build(BaseMotorController motor) {
      double kP =
          new LinearQuadraticRegulator<>(
                  LinearSystemId.identifyVelocitySystem(kV, kA),
                  VecBuilder.fill(maxError),
                  VecBuilder.fill(maxVoltage),
                  RobotMap.TALON_FX_LOOP)
              .getK()
              .get(0, 0);
      kP *= 1023.0 / maxVoltage * unitConversion;
      SmartDashboard.putNumber("Intake kP", kP);
      kV *= 1023.0 / maxVoltage * unitConversion;
      SmartDashboard.putNumber("Intake kV", kV);
      kS /= maxVoltage;
      return new MotorVelocitySystem(motor, kS, kV, unitConversion, kP, maxError);
    }
  }
}
