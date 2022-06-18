package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSFalcon;


public class Intake {
    private static Intake instance;
    private DoubleSolenoid intake;
    private HSFalcon roller;

    private Intake() {
        intake = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.INTAKE_FORWARD ,RobotMap.INTAKE_BACKWARD);
        roller = new HSFalcon(RobotMap.INTAKE_MOTOR);
        initMotor();
    }

    public void initMotor() {
        roller.configFactoryDefault();
        roller.configStatorCurrentLimit(new StatorCurrentLimitConfiguration());
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