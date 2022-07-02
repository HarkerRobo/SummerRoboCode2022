package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.LinearSystemRegulationLoop;
import harkerrobolib.wrappers.HSFalcon;

public class Climber extends SubsystemBase{
    private static Climber instance;

    private HSFalcon left;
    private HSFalcon right;
    private DoubleSolenoid climber;
    private DigitalInput rightLimitSwitch;
    private DigitalInput leftLimitSwitch;

    private static final double CURRENT_CONTINUOUS = 40;
    private static final double CURRENT_PEAK = 45;
    private static final double CURRENT_PEAK_DUR = 0.5;

    private static final double RIGHT_kS = 0;
    private static final double RIGHT_kV = 0;
    private static final double RIGHT_kA = 0;
    private static final double RIGHT_MODEL_STANDARD_DEVIATION = 0.5;
    private static final double RIGHT_ENCODER_STANDARD_DEVIATION = 0.5;
    private static final double RIGHT_MAX_ERROR = 1;
    private static final boolean RIGHT_INVERT = false;

    private static final double LEFT_kS = 0;
    private static final double LEFT_kV = 0;
    private static final double LEFT_kA = 0;
    private static final double LEFT_MODEL_STANDARD_DEVIATION = 0.5;
    private static final double LEFT_ENCODER_STANDARD_DEVIATION = 0.5;
    private static final double LEFT_MAX_ERROR = 1;
    private static final boolean LEFT_INVERT = true;

    private LinearSystemRegulationLoop leftPositionLoop;
    private LinearSystemRegulationLoop rightPositionLoop;

    private Climber() {
        right = new HSFalcon(RobotMap.RIGHT_CLIMBER, RobotMap.CANBUS);
        left = new HSFalcon(RobotMap.LEFT_CLIMBER, RobotMap.CANBUS);
        climber = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.CLIMBER_FORWARD, RobotMap.CLIMBER_BACKWARD);
        rightLimitSwitch = new DigitalInput(RobotMap.CLIMBER_RIGHT_LIMIT_SWTICH);
        leftLimitSwitch = new DigitalInput(RobotMap.CLIMBER_LEFT_LIMIT_SWITCH);
        leftPositionLoop = new LinearSystemRegulationLoop(LinearSystemId.identifyPositionSystem(LEFT_kV, LEFT_kA), LEFT_MODEL_STANDARD_DEVIATION, LEFT_ENCODER_STANDARD_DEVIATION, LEFT_MAX_ERROR, RobotMap.MAX_MOTOR_VOLTAGE);
        rightPositionLoop = new LinearSystemRegulationLoop(LinearSystemId.identifyPositionSystem(RIGHT_kV, RIGHT_kA), RIGHT_MODEL_STANDARD_DEVIATION, RIGHT_ENCODER_STANDARD_DEVIATION, RIGHT_MAX_ERROR, RobotMap.MAX_MOTOR_VOLTAGE);
        initRightMotor();
        initLeftMotor();

    }

    public void initRightMotor() {
        right.configFactoryDefault();
        right.setInverted(RIGHT_INVERT);
        right.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, CURRENT_CONTINUOUS, CURRENT_PEAK, CURRENT_PEAK_DUR));
        right.setNeutralMode(NeutralMode.Brake);
        right.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
        right.configVoltageMeasurementFilter(16);
        right.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, RobotMap.CLIMBER_RIGHT_LIMIT_SWTICH);

    }

    public void initLeftMotor() {
        left.configFactoryDefault();
        left.setInverted(LEFT_INVERT);
        left.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, CURRENT_CONTINUOUS, CURRENT_PEAK, CURRENT_PEAK_DUR));
        left.setNeutralMode(NeutralMode.Brake);
        left.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
        left.configVoltageMeasurementFilter(16);
        left.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, RobotMap.CLIMBER_LEFT_LIMIT_SWITCH);
    }

    public void setRightClimberPos(double pos) {
        right.setVoltage(rightPositionLoop.updateAndPredict(pos, getRightClimberPos()));
    }

    public void setLeftClimberPos(double pos) {
        left.setVoltage(leftPositionLoop.updateAndPredict(pos, getLeftClimberPos()));
    }

    public boolean limitSwitchHit() {
        return !rightLimitSwitch.get() && !leftLimitSwitch.get();
    }

    public void setBothClimberPos(double pos) {
        setRightClimberPos(pos);
        setLeftClimberPos(pos);
    }

    public double getRightClimberPos() {
        return right.getSelectedSensorPosition();
    }

    public double getLeftClimberPos() {
        return left.getSelectedSensorPosition();
    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }
}
