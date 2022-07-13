package frc.robot;

import harkerrobolib.wrappers.XboxGamepad;

public class OI {
    private static OI instance;
    private XboxGamepad operator;
    private XboxGamepad driver;
    
    public static final double DEFAULT_DEADBAND = 0.15;
    

    private OI() {
        operator = new XboxGamepad(RobotMap.OPERATOR_ID);
        driver = new XboxGamepad(RobotMap.DRIVER_ID);
        initBindings();
    }

    public void initBindings() {
    }

    public XboxGamepad getDriverGamepad() {
        return driver;
    }

    public XboxGamepad getOperatorGamepad() {
        return operator;
    }

    public static OI getInstance() {
        if(instance == null) {
            instance = new OI();
        }
        return instance;
    }
}
