package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import harkerrobolib.wrappers.HSFalcon;

public class HSFalconBuilder {
    NeutralMode neutralMode = NeutralMode.Brake;
    boolean invert = false;
    SensorVelocityMeasPeriod velocityMeasPeriod = SensorVelocityMeasPeriod.Period_10Ms;
    int voltageFilter = 16;
    StatorCurrentLimitConfiguration stator;
    SupplyCurrentLimitConfiguration supply;

    public HSFalconBuilder neutralMode(NeutralMode neutralMode){
        this.neutralMode = neutralMode;
        return this;
    }

    public HSFalconBuilder invert(boolean invert) {
        this.invert = invert;
        return this;
    }

    public HSFalconBuilder velocityMeasurementPeriod(SensorVelocityMeasPeriod period){
        velocityMeasPeriod = period;
        return this;
    }

    public HSFalconBuilder voltageFilter(int voltageFilter){
        this.voltageFilter = voltageFilter;
        return this;
    }

    public HSFalconBuilder statorLimit(double peak, double sustained, double peakdur) {
        stator = new StatorCurrentLimitConfiguration(true, sustained, peak, peakdur);
        supply = null;
        return this;
    }

    public HSFalconBuilder supplyLimit(double peak, double sustained, double peakdur) {
        supply = new SupplyCurrentLimitConfiguration(true, sustained, peak, peakdur);
        stator = null;
        return this;
    }
    
    public HSFalcon build(int deviceID, String canbus) {
        HSFalcon falcon =  new HSFalcon(deviceID, canbus);
        falcon.configFactoryDefault();
        falcon.setNeutralMode(neutralMode);
        falcon.setInverted(invert);
        falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        falcon.configVelocityMeasurementPeriod(velocityMeasPeriod);
        falcon.configVoltageMeasurementFilter(voltageFilter);
        if(stator != null) falcon.configStatorCurrentLimit(stator);
        if(supply != null) falcon.configSupplyCurrentLimit(supply);
        return falcon;
    }
}
