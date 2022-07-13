package frc.robot.util;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import frc.robot.RobotMap;

public class LinearSystemRegulationLoop {
    private LinearSystemLoop loop;
    private LinearQuadraticRegulator controller;
    private LinearSystem plant;
    private KalmanFilter observer;
    private double kS;
    private double kG;
    private boolean arm;

    public LinearSystemRegulationLoop(LinearSystem plant, double  modelStdDevs, double measurementStdDevs,
        double error, double controlEffortTolerance, double kS, double kG, boolean arm) {
        this.plant = plant;
        observer = new KalmanFilter<>(Nat.N1(), Nat.N1(), plant, VecBuilder.fill(modelStdDevs), 
            VecBuilder.fill(measurementStdDevs), RobotMap.ROBOT_LOOP);

        controller = new LinearQuadraticRegulator<>(plant, VecBuilder.fill(error), 
            VecBuilder.fill(controlEffortTolerance), RobotMap.ROBOT_LOOP);
            
        loop = new LinearSystemLoop<>(plant, controller, observer, RobotMap.MAX_MOTOR_VOLTAGE, RobotMap.ROBOT_LOOP);

        this.kS = kS;
        this.kG = kG;
        this.arm = arm;
    }

    public LinearSystemRegulationLoop(LinearSystem plant, double  modelStdDevs, double measurementStdDevs,
        double error, double controlEffortTolerance, double kS) {
        this(plant, modelStdDevs, measurementStdDevs, error, controlEffortTolerance, kS, 0.0, false);
    }

    public double updateAndPredict(double setpoint, double systemOutput) {
        loop.setNextR(setpoint);
        loop.correct(VecBuilder.fill(systemOutput));
        loop.predict(RobotMap.ROBOT_LOOP);
        double output = loop.getU(0);
        output += Math.signum(output) * kS;
        if(arm) output += kG * Math.cos(Math.toRadians(loop.getXHat(0)));
        else output += kG;
        return output;
    }

    public double getSetpoint() {
        return loop.getNextR(0);
    }

    public double getError() {
        return loop.getError(0);
    }
}
