package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import harkerrobolib.wrappers.HSFalcon;

public class HSFalconConfigurator {
    public static void configure(HSFalcon falcon, NeutralMode neutralMode, boolean invert, double[] currentLimiting, boolean stator,
        SensorVelocityMeasPeriod velocityMeasPeriod, int voltageFilter) {
        falcon.configFactoryDefault();
        falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        falcon.setNeutralMode(neutralMode);
        falcon.configVelocityMeasurementPeriod(velocityMeasPeriod);
        falcon.configVoltageMeasurementFilter(voltageFilter);
        if(stator)
            falcon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(currentLimiting));
        else
            falcon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(currentLimiting));
    }

    public static void configure(HSFalcon falcon, boolean invert, double[] currentLimiting, boolean stator) {
        configure(falcon, NeutralMode.Brake, invert, currentLimiting, stator, SensorVelocityMeasPeriod.Period_10Ms, 16);
    }
}
