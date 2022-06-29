package frc.robot.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import harkerrobolib.wrappers.HSTalon;

public class HSTalonConfigurator {
    public static void configure(HSTalon talon, NeutralMode neutralMode, boolean invert) {
        talon.configFactoryDefault();
        talon.setNeutralMode(neutralMode);
        talon.setInverted(invert);
        TalonFXConfiguration configuration;
    }
}
