package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSFalcon;


public class Hood extends SubsystemBase{
    private static Hood instance;
    
    private HSFalcon hood;

    private static final double HOOD_CURRENT_CONTINUOUS = 10;
    private static final double HOOD_CURRENT_PEAK = 10;
    private static final double HOOD_CURRENT_PEAK_DUR = 0.05;
    private static final int HOOD_MAX_DEGREE = 26;
    private static final int HOOD_MIN_DEGREE = 0;

    private Hood() {
        hood = new HSFalcon(RobotMap.HOOD_ID, RobotMap.CANBUS);
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
        
    }

    public static Hood getInstance() {
        if (instance == null) instance = new Hood();
        return instance;
    }
    
}
