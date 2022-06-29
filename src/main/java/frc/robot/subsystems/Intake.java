package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.LinearSystemRegulationLoop;
import frc.robot.util.Units;
import harkerrobolib.wrappers.HSFalcon;


public class Intake extends SubsystemBase {
    private static Intake instance;
    private DoubleSolenoid intake;
    private HSFalcon roller;

    public static final double MAX_ROLLER_SPEED = 60; //rotations/sec
    public static final double INTAKE_GEAR_RATIO = 5.0/3.0;

    private static final double CONTINUOUS_CURRENT_LIMIT = 30;
    private static final double PEAK_CURRENT = 40;
    private static final double PEAK_DUR = 0.1;
    private static final boolean INVERT = true;

    private static final double kS = 0; // tune later
    private static final double kV = 0;
    private static final double kA = 0;

    private static final double MAX_ERROR = 1;  
    private static final double MODEL_STANDARD_DEVIATION = 0.5;
    private static final double ENCODER_STANDARD_DEVIATION = 0.035;

    private LinearSystemRegulationLoop velocityLoop;

    private State currIntakeState;

    public static enum State {
        INTAKE,
        NEUTRAL,
        OUTTAKE
    }

    private Intake() {
        intake = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.INTAKE_FORWARD ,RobotMap.INTAKE_BACKWARD);
        roller = new HSFalcon(RobotMap.INTAKE_MOTOR);
        velocityLoop = new LinearSystemRegulationLoop(LinearSystemId.identifyVelocitySystem(kV, kA), MODEL_STANDARD_DEVIATION, ENCODER_STANDARD_DEVIATION, MAX_ERROR, RobotMap.MAX_MOTOR_VOLTAGE);
        initMotor();
    }

    public void initMotor() {
        roller.configFactoryDefault();
        roller.setInverted(INVERT);
        roller.selectProfileSlot(RobotMap.DEFAULT_SLOT_ID, RobotMap.DEFAULT_LOOP_ID);
        roller.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.DEFAULT_LOOP_ID);
        roller.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, CONTINUOUS_CURRENT_LIMIT, PEAK_CURRENT, PEAK_DUR));
    }

    public void setForward() {
        intake.set(DoubleSolenoid.Value.kForward);
    }
    
    public void setBackward() {
        intake.set(DoubleSolenoid.Value.kReverse);
    }

    public void toggle() {
        if(intake.get() == Value.kOff)
            setForward();
        else
            intake.toggle();
    }

    public void setRollerOutput(double rollerOutput) {
        roller.setVoltage(velocityLoop.updateAndPredict(rollerOutput, getIntakeRPS()) + Math.signum(rollerOutput) * kS);
    }

    public State getCurrIntakeState() {
        return currIntakeState;
    }

    public void setCurrIntakeState(State s) {
        currIntakeState = s;
    }
    
    public double getIntakeRPS() {
        return roller.getSelectedSensorVelocity() / INTAKE_GEAR_RATIO * Units.FALCON_VELOCITY_TO_ROT_PER_SECOND;
    }
    
    public HSFalcon getRollerMotor() {
        return roller;
    }

    public static Intake getInstance() {
        if (instance == null)
            instance = new Intake();
        return instance;
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Intake");
        builder.addDoubleProperty("Current Intake Roller Sensor Velocity", () -> roller.getSelectedSensorVelocity(), null);
        builder.addDoubleProperty("Current Intake Roller RPS", () -> getIntakeRPS(), null);
    } 
}