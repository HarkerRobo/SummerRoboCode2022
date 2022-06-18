package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import harkerrobolib.wrappers.HSMotorController;
import harkerrobolib.wrappers.XboxGamepad;

public class OI {
    private static OI instance;
    private XboxGamepad operator;
    private XboxGamepad driver;

    private OI() {
        operator = new XboxGamepad(RobotMap.OPERATOR_ID);
        driver = new XboxGamepad(RobotMap.DRIVER_ID);
    }

    public static OI getInstance() {
        if(instance == null) {
            instance = new OI();
        }
        return instance;
    }
}
