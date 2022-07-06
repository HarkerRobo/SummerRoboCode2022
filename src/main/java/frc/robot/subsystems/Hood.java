package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.LinearSystemRegulationLoop;
import frc.robot.util.Units;
import harkerrobolib.wrappers.HSFalcon;


public class Hood extends SubsystemBase{
    private static Hood instance;
    
    private HSFalcon hood;

    private static final double HOOD_CURRENT_CONTINUOUS = 10;
    private static final double HOOD_CURRENT_PEAK = 10;
    private static final double HOOD_CURRENT_PEAK_DUR = 0.05;
    private static final int HOOD_RANGE = 23;

    private static final double kS = 0;
    private static final double kV = 0;
    private static final double kA = 0;
    private static final double kG = 0.087132;

    private static final double HOOD_GEAR_RATIO = 180; // needs to be updated

    private static final double MAX_ERROR = 5;  
    private static final double MODEL_STANDARD_DEVIATION = 0.5;
    private static final double ENCODER_STANDARD_DEVIATION = 0.035;

    private static final double HOOD_STALLING_CURRENT = 10;
    
    private boolean isHoodZeroed;

    private LinearSystemRegulationLoop positionLoop;

    private Hood() {
        hood = new HSFalcon(RobotMap.HOOD_ID, RobotMap.CANBUS);
        positionLoop = new LinearSystemRegulationLoop(LinearSystemId.identifyPositionSystem(kV, kA), MODEL_STANDARD_DEVIATION, ENCODER_STANDARD_DEVIATION, MAX_ERROR, RobotMap.MAX_MOTOR_VOLTAGE);
        isHoodZeroed = false;
    }

    public void initMotors() {
        hood.configFactoryDefault();
        hood.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, HOOD_CURRENT_CONTINUOUS, HOOD_CURRENT_PEAK, HOOD_CURRENT_PEAK_DUR));
        hood.setNeutralMode(NeutralMode.Brake);
        hood.configForwardSoftLimitEnable(true);
        hood.configForwardSoftLimitThreshold(HOOD_RANGE * HOOD_GEAR_RATIO * Units.DEGREES_TO_ENCODER_TICKS);
        hood.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
        hood.configVoltageMeasurementFilter(16);
    }

    public void setHoodPosition(double position) {
        hood.setVoltage(positionLoop.updateAndPredict(position, getHoodPosition()) + kS * Math.signum(getHoodPosition()-position) + kG); // potentially change
    }

    public void setHoodPercentOutput(double percentOutput) {
        hood.set(ControlMode.PercentOutput, percentOutput);
    }

    public void setHoodEncoderZero() {
        hood.setSelectedSensorPosition(0);
    }

    public void setIsHoodZeroed(boolean zeroed) {
        isHoodZeroed = zeroed;
    }

    public double getHoodPosition() {
        return hood.getSelectedSensorPosition() * Units.ENCODER_TICKS_TO_DEGREES / HOOD_GEAR_RATIO;
    }

    public boolean isHoodZero() {
        return hood.getStatorCurrent() >= HOOD_STALLING_CURRENT;
    }

    public static Hood getInstance() {
        if (instance == null) instance = new Hood();
        return instance;
    }
    
}
