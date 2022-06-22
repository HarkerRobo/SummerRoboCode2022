package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSFalcon;

public class Indexer extends SubsystemBase{
    private static Indexer instance;
    private HSFalcon top;
    private HSFalcon bottom;
    private DigitalInput topProximity;
    private DigitalInput bottomProximity;

    private static final double BOTTOM_kP = 1; // tune later
    private static final double BOTTOM_kS = 0.02;
    private static final double TOP_kP = 1; // tune later
    private static final double TOP_kS = 0.02;

    private static final double INDEXER_CURRENT_CONTINUOUS = 30;
    private static final double INDEXER_CURRENT_PEAK = 60;
    private static final double INDEXER_CURRENT_PEAK_DUR = 0.2;
    private static final boolean TOP_INVERT = true;
    private static final boolean BOTTOM_INVERT = false;


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
        top.configNominalOutputForward(TOP_kS);

        bottom.configFactoryDefault();
        bottom.setInverted(BOTTOM_INVERT);
        bottom.selectProfileSlot(RobotMap.DEFAULT_SLOT_ID, RobotMap.DEFAULT_LOOP_ID);
        bottom.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.DEFAULT_LOOP_ID);
        bottom.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, INDEXER_CURRENT_CONTINUOUS, INDEXER_CURRENT_PEAK, INDEXER_CURRENT_PEAK_DUR));
        bottom.configClosedloopRamp(0.3);
        bottom.configOpenloopRamp(0.3);
        bottom.config_kP(RobotMap.DEFAULT_SLOT_ID, BOTTOM_kP);
        bottom.configNominalOutputForward(BOTTOM_kS);
    }

    public void setTopOutput(double topOutput) {
        top.set(ControlMode.PercentOutput, topOutput);
    }

    public void setBottomOutput(double bottomOutput) {
        bottom.set(ControlMode.PercentOutput, bottomOutput);
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

    public static Indexer getInstance() {
        if(instance == null) {
            instance = new Indexer();
        }
        return instance;
    }
}
