package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.LinearSystemRegulationLoop;
import frc.robot.util.Units;
import harkerrobolib.wrappers.HSFalcon;

public class Indexer extends SubsystemBase{
    private static Indexer instance;
    private HSFalcon top;
    private HSFalcon bottom;
    private DigitalInput topProximity;
    private DigitalInput bottomProximity;

    private static final double INDEXER_CURRENT_CONTINUOUS = 30;
    private static final double INDEXER_CURRENT_PEAK = 60;
    private static final double INDEXER_CURRENT_PEAK_DUR = 0.2;
    private static final boolean TOP_INVERT = true;
    private static final boolean BOTTOM_INVERT = false;

    private static final double INDEXER_TOP_GEAR_RATIO = 5.0;
    private static final double INDEXER_BOTTOM_GEAR_RATIO = 1.5;

    //tune later
    private static final double TOP_kS = 0;
    private static final double TOP_kV = 0;
    private static final double TOP_kA = 0;
    private static final double BOTTOM_kS = 0;
    private static final double BOTTOM_kV = 0;
    private static final double BOTTOM_kA = 0;

    //tune later
    private static final double TOP_MAX_ERROR = 0.5;
    private static final double TOP_MODEL_STANDARD_DEVIATION = 0.5;
    private static final double TOP_ENCODER_STANDARD_DEVIATION = 0.035;

    private static final double BOTTOM_MAX_ERROR = 0.5;
    private static final double BOTTOM_MODEL_STANDARD_DEVIATION = 0.5;
    private static final double BOTTOM_ENCODER_STANDARD_DEVIATION = 0.035;

    private LinearSystemRegulationLoop bottomVelocityLoop;
    private LinearSystemRegulationLoop topVelocityLoop;

    private Indexer() {
        top = new HSFalcon(RobotMap.INDEXER_TOP, RobotMap.CANBUS);
        bottom = new HSFalcon(RobotMap.INDEXER_BOTTOM, RobotMap.CANBUS);
        topProximity = new DigitalInput(RobotMap.TOP_PROXIMITY);
        bottomProximity = new DigitalInput(RobotMap.BOTTOM_PROXIMITY);
        topVelocityLoop = new LinearSystemRegulationLoop(LinearSystemId.identifyVelocitySystem(TOP_kV, TOP_kA), TOP_MODEL_STANDARD_DEVIATION, TOP_ENCODER_STANDARD_DEVIATION, TOP_MAX_ERROR, RobotMap.MAX_MOTOR_VOLTAGE);
        bottomVelocityLoop = new LinearSystemRegulationLoop(LinearSystemId.identifyVelocitySystem(BOTTOM_kV, BOTTOM_kA), BOTTOM_MODEL_STANDARD_DEVIATION, BOTTOM_ENCODER_STANDARD_DEVIATION, BOTTOM_MAX_ERROR, RobotMap.MAX_MOTOR_VOLTAGE);
        initMotors();
    }

    private void initMotors() {
        top.configFactoryDefault();
        top.setInverted(TOP_INVERT);
        top.selectProfileSlot(RobotMap.DEFAULT_SLOT_ID, RobotMap.DEFAULT_LOOP_ID);
        top.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.DEFAULT_SLOT_ID);
        top.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, INDEXER_CURRENT_CONTINUOUS, INDEXER_CURRENT_PEAK, INDEXER_CURRENT_PEAK_DUR));
        top.configClosedloopRamp(0.3);
        top.configOpenloopRamp(0.3);

        bottom.configFactoryDefault();
        bottom.setInverted(BOTTOM_INVERT);
        bottom.selectProfileSlot(RobotMap.DEFAULT_SLOT_ID, RobotMap.DEFAULT_LOOP_ID);
        bottom.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.DEFAULT_LOOP_ID);
        bottom.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, INDEXER_CURRENT_CONTINUOUS, INDEXER_CURRENT_PEAK, INDEXER_CURRENT_PEAK_DUR));
        bottom.configClosedloopRamp(0.3);
        bottom.configOpenloopRamp(0.3);
    }

    public void setTopOutput(double topOutput) {
        top.setVoltage(topVelocityLoop.updateAndPredict(topOutput, Units.wheelRotsToMeter(4.0) * getTopRPS()) + TOP_kS * Math.signum(topOutput));
    }

    public void setBottomOutput(double bottomOutput) {
        bottom.setVoltage(bottomVelocityLoop.updateAndPredict(bottomOutput, Units.wheelRotsToMeter(3.0) * getBottomRPS()) + BOTTOM_kS * Math.signum(bottomOutput));

    }

    public void setBothOutput(double output) {
        setTopOutput(output);
        setBottomOutput(output);
    }

    public boolean isBallInTop() {
        return !topProximity.get();
    }

    public boolean isBallInBottom() {
        return !bottomProximity.get();
    }

    public double getTopRPS() {
        return top.getSelectedSensorVelocity() * Units.FALCON_VELOCITY_TO_ROT_PER_SECOND / INDEXER_TOP_GEAR_RATIO;
    }

    public double getBottomRPS() {
        return bottom.getSelectedSensorVelocity() * Units.FALCON_VELOCITY_TO_ROT_PER_SECOND / INDEXER_BOTTOM_GEAR_RATIO;
    }

    public static Indexer getInstance() {
        if(instance == null) {
            instance = new Indexer();
        }
        return instance;
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Indexer");
        builder.addDoubleProperty("Current Indexer Top Velocity",  () -> getTopRPS(), null);
        builder.addDoubleProperty("Current Indexer Bottom Velocity", () -> getBottomRPS(), null);
        builder.addDoubleProperty("Current Indexer Top Sensor Velocity", () -> top.getSelectedSensorVelocity(), null);
        builder.addDoubleProperty("Current Indexer Bottom Sensor Velocity", () -> bottom.getSelectedSensorVelocity(), null);        
    }
}
