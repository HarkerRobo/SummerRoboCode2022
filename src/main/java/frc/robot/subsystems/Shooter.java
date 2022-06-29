package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.LinearSystemRegulationLoop;
import frc.robot.util.Units;
import harkerrobolib.wrappers.HSFalcon;

public class Shooter extends SubsystemBase {
    private static Shooter instance;

    private HSFalcon master;
    private HSFalcon follower;

    private LinearSystemRegulationLoop velocityLoop;

    private static final boolean MASTER_INVERT = true; 
    private static final boolean FOLLOWER_INVERT = false;

    private static final double CURRENT_CONTINUOUS = 40;
    private static final double CURRENT_PEAK = 45;
    private static final double CURRENT_PEAK_DUR = 0.5;

    private static final double kS = 0.0;
    private static final double kV = 0.0; // TODO: update shooter Linear System constants
    private static final double kA = 0.0;
    private static final double ENCODER_STANDARD_DEVIATION = 0.0;
    private static final double MODEL_STANDARD_DEVIATION = 0.0;
    private static final double MAX_ERROR = 1.0;

    private static final double SHOOTER_GEAR_RATIO = 1.5; // TODO: update shooter gear ratio

    private Shooter() {
        master = new HSFalcon(RobotMap.SHOOTER_MASTER, RobotMap.CANBUS);
        follower = new HSFalcon(RobotMap.SHOOTER_FOLLOWER, RobotMap.CANBUS);
        velocityLoop = new LinearSystemRegulationLoop(LinearSystemId.identifyVelocitySystem(kV, kA), MODEL_STANDARD_DEVIATION, ENCODER_STANDARD_DEVIATION, MAX_ERROR, RobotMap.MAX_MOTOR_VOLTAGE);
        initMotors();
    }

    private void initMotors() {
        follower.follow(master);
        master.configFactoryDefault();
        master.setNeutralMode(NeutralMode.Coast);
        follower.setNeutralMode(NeutralMode.Coast);
        master.setInverted(MASTER_INVERT);
        follower.setInverted(FOLLOWER_INVERT);
        master.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, CURRENT_CONTINUOUS, CURRENT_PEAK, CURRENT_PEAK_DUR));
        master.configForwardSoftLimitEnable(false);
        master.configReverseSoftLimitEnable(false);
        master.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
        master.configVoltageMeasurementFilter(16);
    }

    public void set(double speed) {
        master.set(ControlMode.PercentOutput, (velocityLoop.updateAndPredict(speed, getCurrentRPS()) + kS * Math.signum(speed)) / RobotMap.MAX_MOTOR_VOLTAGE);
    }

    public double getCurrentRPS() {
        return master.getSelectedSensorVelocity() * Units.FALCON_VELOCITY_TO_ROT_PER_SECOND / SHOOTER_GEAR_RATIO;
    }

    public static Shooter getInstance() {
        if(instance == null)
            instance = new Shooter();
        return instance;
    }
}
