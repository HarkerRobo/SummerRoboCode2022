package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

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
    private static final int HOOD_MAX_DEGREE = 26;
    private static final int HOOD_MIN_DEGREE = 0;

    private static final double kS = 0;
    private static final double kV = 0;
    private static final double kA = 0;
    private static final double kG = 0.087132;

    private static final double HOOD_GEAR_RATIO = 180; // needs to be updated

    private static final double MAX_ERROR = 5;  
    private static final double MODEL_STANDARD_DEVIATION = 0.5;
    private static final double ENCODER_STANDARD_DEVIATION = 0.035;


    private LinearSystemRegulationLoop positionLoop;

    private Hood() {
        hood = new HSFalcon(RobotMap.HOOD_ID, RobotMap.CANBUS);
        positionLoop = new LinearSystemRegulationLoop(LinearSystemId.identifyPositionSystem(kV, kA), MODEL_STANDARD_DEVIATION, ENCODER_STANDARD_DEVIATION, MAX_ERROR, RobotMap.MAX_MOTOR_VOLTAGE);
    }

    public void initMotors() {
        hood.configFactoryDefault();
        hood.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, HOOD_CURRENT_CONTINUOUS, HOOD_CURRENT_PEAK, HOOD_CURRENT_PEAK_DUR));
        hood.setNeutralMode(NeutralMode.Brake);
        hood.configForwardSoftLimitEnable(true);
        hood.configReverseSoftLimitEnable(true);
        hood.configForwardSoftLimitThreshold(HOOD_MAX_DEGREE);
        hood.configReverseSoftLimitThreshold(HOOD_MIN_DEGREE);
    }

    public void setHoodPosition(double position) {
        hood.setVoltage(positionLoop.updateAndPredict(position, getHoodPosition()) + kS * Math.signum(getHoodPosition()-position) + kG);
    }

    public double getHoodPosition() {
        return hood.getSelectedSensorPosition() * Units.ENCODER_TICKS_TO_DEGREES / HOOD_GEAR_RATIO;
    }

    public static Hood getInstance() {
        if (instance == null) instance = new Hood();
        return instance;
    }
    
}
