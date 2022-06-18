package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSFalcon;


public class Intake {
    private static Intake instance;
    private DoubleSolenoid intake;
    private HSFalcon roller;

    private static final double CONTINUOUS_CURRENT_LIMIT = 30;
    private static final double PEAK_CURRENT = 40;
    private static final double PEAK_DUR = 0.1;

    private Intake() {
        intake = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.INTAKE_FORWARD ,RobotMap.INTAKE_BACKWARD);
        roller = new HSFalcon(RobotMap.INTAKE_MOTOR);
        initMotor();
    }

    public void initMotor() {
        roller.configFactoryDefault();
        roller.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, CONTINUOUS_CURRENT_LIMIT, PEAK_CURRENT, PEAK_DUR));
    }

    public void setForward() {
        intake.set(DoubleSolenoid.Value.kForward);
    }
    
    public void setBackward() {
        intake.set(DoubleSolenoid.Value.kReverse);
    }

    public void setRollerOutput(double rollerOutput) {
        roller.set(ControlMode.PercentOutput, rollerOutput);
    }
    
    public static Intake getInstance() {
        if (instance == null)
            instance = new Intake();
        return instance;
    }
}