package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.HSFalconConfigurator;
import frc.robot.util.LinearSystemRegulationLoop;
import frc.robot.util.Units;
import harkerrobolib.wrappers.HSFalcon;


public class Hood extends SubsystemBase{
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

    private static final double MAX_ERROR = 5;  
    private static final double MODEL_STANDARD_DEVIATION = 0.5;
    private static final double ENCODER_STANDARD_DEVIATION = 0.035;

    private static final double STALLING_CURRENT = 10;
    
    private boolean isHoodZeroed;

    private LinearSystemRegulationLoop positionLoop;

    private Hood() {
        hood = new HSFalcon(RobotMap.HOOD_ID, RobotMap.CANBUS);
        positionLoop = new LinearSystemRegulationLoop(LinearSystemId.identifyPositionSystem(kV, kA), MODEL_STANDARD_DEVIATION, ENCODER_STANDARD_DEVIATION, MAX_ERROR, RobotMap.MAX_MOTOR_VOLTAGE, kS);
        isHoodZeroed = false;
    }

    public void initMotors() {
        HSFalconConfigurator.configure(hood, INVERT, new double[]{1.0, CURRENT_CONTINUOUS, CURRENT_PEAK, CURRENT_PEAK_DUR}, true);
        hood.configForwardSoftLimitEnable(true);
        hood.configForwardSoftLimitThreshold(RANGE * GEAR_RATIO * Units.DEGREES_TO_ENCODER_TICKS);
    }

    public void setHoodPosition(double position) {
        hood.setVoltage(positionLoop.updateAndPredict(position, getHoodPosition()) + kS * Math.signum(getHoodPosition()-position) + kG); // potentially change
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
