package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.Units;
import harkerrobolib.wrappers.HSFalcon;


public class Intake extends SubsystemBase {
    private static Intake instance;
    private DoubleSolenoid intake;
    private HSFalcon roller;

    public static final double MAX_ROLLER_SPEED = 60; //rotations/sec
    public static final double INTAKE_GEAR_RATIO = 0.6;

    private static final double CONTINUOUS_CURRENT_LIMIT = 30;
    private static final double PEAK_CURRENT = 40;
    private static final double PEAK_DUR = 0.1;
    private static final boolean INVERT = true;

    private static final double kP = 1;
    private static final double kF = 0.02;

    private State currIntakeState;

    public static enum State {
        INTAKE,
        NEUTRAL,
        OUTTAKE
    }

    private Intake() {
        intake = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.INTAKE_FORWARD ,RobotMap.INTAKE_BACKWARD);
        roller = new HSFalcon(RobotMap.INTAKE_MOTOR);
        initMotor();
    }

    public void initMotor() {
        roller.configFactoryDefault();
        roller.setInverted(INVERT);
        roller.selectProfileSlot(RobotMap.DEFAULT_SLOT_ID, RobotMap.DEFAULT_LOOP_ID);
        roller.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.DEFAULT_LOOP_ID);
        roller.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, CONTINUOUS_CURRENT_LIMIT, PEAK_CURRENT, PEAK_DUR));
        roller.config_kP(RobotMap.DEFAULT_SLOT_ID, kP);
        roller.config_kF(RobotMap.DEFAULT_SLOT_ID, kF);
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
        roller.set(ControlMode.Velocity, rollerOutput * INTAKE_GEAR_RATIO / Units.FALCON_ENCODER_TICKS);
    }

    public State getCurrIntakeState() {
        return currIntakeState;
    }

    public void setCurrIntakeState(State s) {
        currIntakeState = s;
    }
    
    public double getIntakeRPS() {
        return roller.getSelectedSensorVelocity() * INTAKE_GEAR_RATIO / Units.FALCON_ENCODER_TICKS;
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
        builder.addDoubleProperty("Intake kP", () -> kP, (double d) -> {roller.config_kP(RobotMap.DEFAULT_SLOT_ID, d);});

    } 
}