package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.Units;
import harkerrobolib.wrappers.HSFalcon;

public class Indexer extends SubsystemBase{
    private static Indexer instance;
    private HSFalcon top;
    private HSFalcon bottom;
    private DigitalInput topProximity;
    private DigitalInput bottomProximity;

    private static final double BOTTOM_kP = 1; // tune later
    private static final double BOTTOM_kF = 0.02;
    private static final double TOP_kP = 1; // tune later
    private static final double TOP_kF = 0.02;

    private static final double INDEXER_CURRENT_CONTINUOUS = 30;
    private static final double INDEXER_CURRENT_PEAK = 60;
    private static final double INDEXER_CURRENT_PEAK_DUR = 0.2;
    private static final boolean TOP_INVERT = true;
    private static final boolean BOTTOM_INVERT = false;

    private static final double INDEXER_TOP_GEAR_RATIO = 2; // replace
    private static final double INDEXER_BOTTOM_GEAR_RATIO = 2;

    private Indexer() {
        top = new HSFalcon(RobotMap.INDEXER_TOP);
        bottom = new HSFalcon(RobotMap.INDEXER_BOTTOM);
        topProximity = new DigitalInput(RobotMap.TOP_PROXIMITY);
        bottomProximity = new DigitalInput(RobotMap.BOTTOM_PROXIMITY);
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
        top.config_kP(RobotMap.DEFAULT_SLOT_ID, TOP_kP);
        top.config_kF(RobotMap.DEFAULT_SLOT_ID, TOP_kF);

        bottom.configFactoryDefault();
        bottom.setInverted(BOTTOM_INVERT);
        bottom.selectProfileSlot(RobotMap.DEFAULT_SLOT_ID, RobotMap.DEFAULT_LOOP_ID);
        bottom.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.DEFAULT_LOOP_ID);
        bottom.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, INDEXER_CURRENT_CONTINUOUS, INDEXER_CURRENT_PEAK, INDEXER_CURRENT_PEAK_DUR));
        bottom.configClosedloopRamp(0.3);
        bottom.configOpenloopRamp(0.3);
        bottom.config_kP(RobotMap.DEFAULT_SLOT_ID, BOTTOM_kP);
        bottom.config_kF(RobotMap.DEFAULT_SLOT_ID, BOTTOM_kF);
    }

    public void setTopOutput(double topOutput) {
        top.set(ControlMode.Velocity, topOutput);
    }

    public void setBottomOutput(double bottomOutput) {
        bottom.set(ControlMode.Velocity, bottomOutput);
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
        builder.addDoubleProperty("Indexer Top kP", () -> TOP_kP, (double d) -> {top.config_kP(RobotMap.DEFAULT_SLOT_ID, d);});
        builder.addDoubleProperty("Indexer Bottom kP", () -> BOTTOM_kP, (double d) -> {bottom.config_kP(RobotMap.DEFAULT_SLOT_ID, d);});
        
    }
}
