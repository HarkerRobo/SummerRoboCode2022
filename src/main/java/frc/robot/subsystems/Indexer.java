package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.fasterxml.jackson.databind.ser.std.NullSerializer;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.ColorSensor;
import frc.robot.util.LinearSystemRegulationLoop;
import frc.robot.util.Units;
import harkerrobolib.wrappers.HSFalcon;

public class Indexer extends SubsystemBase{
    private static Indexer instance;
    private HSFalcon top;
    private HSFalcon bottom;
    private DigitalInput topProximity;
    private DigitalInput bottomProximity;
    private ColorSensor colorSensor;

    private static final double INDEXER_CURRENT_CONTINUOUS = 30;
    private static final double INDEXER_CURRENT_PEAK = 60;
    private static final double INDEXER_CURRENT_PEAK_DUR = 0.2;
    private static final boolean TOP_INVERT = true;
    private static final boolean BOTTOM_INVERT = false;

    private static final double INDEXER_TOP_GEAR_RATIO = 5.0;
    private static final double INDEXER_BOTTOM_GEAR_RATIO = 1.5;

    //tune later
    private static final double TOP_kS = 0.14855;
    private static final double TOP_kV = 2.5874;
    private static final double TOP_kA = 0.049339;
    private static final double BOTTOM_kS = 0.050444;
    private static final double BOTTOM_kV = 1.704;
    private static final double BOTTOM_kA = 0.026034;

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
        topVelocityLoop = new LinearSystemRegulationLoop(LinearSystemId.identifyVelocitySystem(TOP_kV, TOP_kA), TOP_MODEL_STANDARD_DEVIATION, TOP_ENCODER_STANDARD_DEVIATION, TOP_MAX_ERROR, RobotMap.MAX_MOTOR_VOLTAGE, TOP_kS);
        bottomVelocityLoop = new LinearSystemRegulationLoop(LinearSystemId.identifyVelocitySystem(BOTTOM_kV, BOTTOM_kA), BOTTOM_MODEL_STANDARD_DEVIATION, BOTTOM_ENCODER_STANDARD_DEVIATION, BOTTOM_MAX_ERROR, RobotMap.MAX_MOTOR_VOLTAGE, BOTTOM_kS);
        colorSensor = new ColorSensor(RobotMap.COLOR_A, RobotMap.COLOR_B, RobotMap.COLOR_PROXIMITY);
        initMotors();
    }

    private void initMotors() {
        
        top.configFactoryDefault();
        top.setInverted(TOP_INVERT);
        top.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, INDEXER_CURRENT_CONTINUOUS, INDEXER_CURRENT_PEAK, INDEXER_CURRENT_PEAK_DUR));

        bottom.configFactoryDefault();
        bottom.setInverted(BOTTOM_INVERT);
        bottom.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, INDEXER_CURRENT_CONTINUOUS, INDEXER_CURRENT_PEAK, INDEXER_CURRENT_PEAK_DUR));
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
        builder.addDoubleProperty("Top Indexer Velocity",  () -> getTopRPS(), null);
        builder.addDoubleProperty("Bottom Indexer Velocity", () -> getBottomRPS(), null);
        builder.addBooleanProperty("Color is red", () -> colorSensor.isRed(), null);
        builder.addBooleanProperty("Color sensor is functioning", () -> colorSensor.isFunctioning(), null);
        // builder.addDoubleProperty("Top Indexer Sensor Velocity", () -> top.getSelectedSensorVelocity(), null);
        // builder.addDoubleProperty("Bottom Indexer Sensor Velocity", () -> bottom.getSelectedSensorVelocity(), null);        
    }
}
