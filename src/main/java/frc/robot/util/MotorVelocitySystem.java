package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.RobotMap;

public class MotorVelocitySystem implements Sendable {
  protected BaseMotorController motor;
  protected double kS, kP, kV, kA, kF, unitConversion, velocitySetpoint, maxError, maxVoltage;

  protected MotorVelocitySystem(
      BaseMotorController motor,
      double kS,
      double kV,
      double kA,
      double unitConversion,
      double maxError, double maxVoltage) {
    this.motor = motor;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.unitConversion = unitConversion;
    this.maxError = maxError;
    this.maxVoltage = maxVoltage;
    this.calculateConstants();
    motor.selectProfileSlot(RobotMap.SLOT_INDEX, 0);
    motor.configAllowableClosedloopError(RobotMap.SLOT_INDEX, maxError / unitConversion);
  }

  public MotorVelocitySystem init() {
    velocitySetpoint = 0;
    this.calculateConstants();
    motor.selectProfileSlot(RobotMap.SLOT_INDEX, 0);
    return this;
  }

  public void calculateConstants() {
    kP = new LinearQuadraticRegulator<>(
      LinearSystemId.identifyVelocitySystem(kV, kA),
      VecBuilder.fill(maxError),
      VecBuilder.fill(maxVoltage),
      RobotMap.TALON_FX_LOOP)
      .getK()
      .get(0, 0);
    System.out.println(kP);
    kP *= 1023.0 / maxVoltage * unitConversion;
    kF = kV * 1023.0 / maxVoltage * unitConversion;
    configConstants();
  }

  public void configConstants() {
    motor.config_kP(RobotMap.SLOT_INDEX, kP);
    motor.config_kF(RobotMap.SLOT_INDEX, kF);
  }

  public void set(double output) {
    velocitySetpoint = output;
    motor.set(
        ControlMode.Velocity,
        output / unitConversion,
        DemandType.ArbitraryFeedForward,
        kS * Math.signum(output) / maxVoltage);
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
    builder.addDoubleProperty("Velocity", () -> getVelocity(), null);
    builder.addDoubleProperty("Vel Setpoint", () -> getVelocitySetpoint(), (a) -> set(a));
    builder.addDoubleProperty("Vel Error", () -> getVelocityError(), null);
    builder.addDoubleProperty("Max Error", () -> maxError, (a) -> {maxError = a; calculateConstants();});
    builder.addDoubleProperty("Max Voltage", () -> maxVoltage, (a) -> {motor.configVoltageCompSaturation(a);maxVoltage=a;calculateConstants();});
    builder.addDoubleProperty("kS", () -> kS, (a) -> kS = a);
    builder.addDoubleProperty("kV", () -> kV, (a) -> {this.kV = a; calculateConstants();});
    builder.addDoubleProperty("kA", () -> kA, (a) -> {this.kA = a; calculateConstants();});
    builder.addDoubleProperty("kP", () -> kP, (a) -> {this.kP = a; configConstants();});
    builder.addDoubleProperty("kF", () -> kF, (a) -> {this.kF = a; configConstants();});
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
      return new MotorVelocitySystem(motor, kS, kV, kA, unitConversion, maxError, maxVoltage);
    }
  }
}
