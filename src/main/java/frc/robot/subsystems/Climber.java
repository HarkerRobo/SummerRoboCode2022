package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.HSFalconBuilder;
import frc.robot.util.loop.PositionControlLoop;
import frc.robot.util.loop.PositionControlLoop.PositionControlLoopBuilder;
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
    private static final double RIGHT_kG = 0;
    private static final double RIGHT_MODEL_POS_STDEV = 0.5;
    private static final double RIGHT_MODEL_VEL_STDEV = 0.5;
    private static final double RIGHT_ENCODER_STDEV = 0.5;
    private static final double RIGHT_POS_MAX_ERROR = 1;
    private static final double RIGHT_VEL_MAX_ERROR = 1;
    private static final boolean RIGHT_INVERT = false;

    private static final double LEFT_kS = 0;
    private static final double LEFT_kV = 0;
    private static final double LEFT_kA = 0;
    private static final double LEFT_kG = 0;
    private static final double LEFT_MODEL_POS_STDEV = 0.5;
    private static final double LEFT_MODEL_VEL_STDEV = 0.5;
    private static final double LEFT_ENCODER_STDEV = 0.5;
    private static final double LEFT_POS_MAX_ERROR = 1;
    private static final double LEFT_VEL_MAX_ERROR = 1;
    private static final boolean LEFT_INVERT = true;

    public static final double UP_HEIGHT = 119000; //TODO
    public static final double MID_HEIGHT = 60000; //TODO
    public static final double DOWN_HEIGHT = 0; //TODO

    private PositionControlLoop leftPositionLoop;
    private PositionControlLoop rightPositionLoop;

    private Climber() {
        right = new HSFalconBuilder()
                    .invert(RIGHT_INVERT)
                    .statorLimit(CURRENT_PEAK, CURRENT_CONTINUOUS, CURRENT_PEAK_DUR)
                    .build(RobotMap.RIGHT_CLIMBER, RobotMap.CANBUS);
        left = new HSFalconBuilder()
                    .invert(LEFT_INVERT)
                    .statorLimit(CURRENT_PEAK, CURRENT_CONTINUOUS, CURRENT_PEAK_DUR)
                    .build(RobotMap.LEFT_CLIMBER, RobotMap.CANBUS);
        climber = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.CLIMBER_FORWARD, RobotMap.CLIMBER_BACKWARD);
        rightLimitSwitch = new DigitalInput(RobotMap.CLIMBER_RIGHT_LIMIT_SWTICH);
        leftLimitSwitch = new DigitalInput(RobotMap.CLIMBER_LEFT_LIMIT_SWITCH);
        leftPositionLoop = new PositionControlLoopBuilder()
                            .motorConstants(LEFT_kS, LEFT_kA, LEFT_kV, LEFT_kG)
                            .standardDeviations(LEFT_MODEL_POS_STDEV, LEFT_MODEL_VEL_STDEV, LEFT_ENCODER_STDEV)
                            .maxError(LEFT_POS_MAX_ERROR, LEFT_VEL_MAX_ERROR)
                            .buildElevatorControlLoop();
        rightPositionLoop = new PositionControlLoopBuilder()
                            .motorConstants(RIGHT_kS, RIGHT_kA, RIGHT_kV, RIGHT_kG)
                            .standardDeviations(RIGHT_MODEL_POS_STDEV, RIGHT_MODEL_VEL_STDEV, RIGHT_ENCODER_STDEV)
                            .maxError(RIGHT_POS_MAX_ERROR, RIGHT_VEL_MAX_ERROR)
                            .buildElevatorControlLoop();
    }

    public void setRightClimberPos(double pos) {
        right.setVoltage(rightPositionLoop.setReferenceAndPredict(pos, 0.0, getRightClimberVel()));
    }

    public void setLeftClimberPos(double pos) {
        left.setVoltage(leftPositionLoop.setReferenceAndPredict(pos, 0.0, getLeftClimberVel()));
    }

    public boolean limitSwitchHit() {
        return !rightLimitSwitch.get() && !leftLimitSwitch.get();
    }

    public void setBothClimberPos(double pos) {
        setRightClimberPos(pos);
        setLeftClimberPos(pos);
    }

    public void setClimberForward() {
        climber.set(DoubleSolenoid.Value.kReverse);
    }

    public void setClimberBackward() {
        climber.set(DoubleSolenoid.Value.kForward);
    }

    public void turnOffMotors() {
        right.set(ControlMode.PercentOutput, 0);
        left.set(ControlMode.PercentOutput, 0);
    }

    public double getRightClimberPos() {
        return right.getSelectedSensorPosition();
    }

    public double getLeftClimberPos() {
        return left.getSelectedSensorPosition();
    }

    public double getRightClimberVel() {
        return right.getSelectedSensorVelocity();
    }

    public double getLeftClimberVel() {
        return left.getSelectedSensorVelocity();
    }

    public PositionControlLoop getLeftControlLoop() {
        return leftPositionLoop;
    }

    public PositionControlLoop getRightControlLoop() {
        return rightPositionLoop;
    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }
}
