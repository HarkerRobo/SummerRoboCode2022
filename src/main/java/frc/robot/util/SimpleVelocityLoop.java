package frc.robot.util;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.RobotMap;

public class SimpleVelocityLoop {
    private LinearSystemLoop<N1, N1, N1> loop;
    private LinearQuadraticRegulator<N1, N1, N1> controller;
    private LinearSystem<N1, N1, N1> plant;
    private KalmanFilter<N1, N1, N1> observer;

    public SimpleVelocityLoop(double kV, double kA, double  modelStdDevs, double measurementStdDevs,
        double velError, double controlEffortTolerance) {
        plant = LinearSystemId.identifyVelocitySystem(kV, kA);
        observer = new KalmanFilter<>(Nat.N1(), Nat.N1(), plant, VecBuilder.fill(modelStdDevs), 
            VecBuilder.fill(measurementStdDevs), RobotMap.ROBOT_LOOP);
        controller = new LinearQuadraticRegulator<>(plant, VecBuilder.fill(velError), 
            VecBuilder.fill(controlEffortTolerance), RobotMap.ROBOT_LOOP);
        loop = new LinearSystemLoop<>(plant, controller, observer, RobotMap.MAX_MOTOR_VOLTAGE, RobotMap.ROBOT_LOOP);
    }

    public double updateAndPredict(double setpoint, double systemOutput) {
        loop.setNextR(setpoint);
        loop.correct(VecBuilder.fill(systemOutput));
        loop.predict(RobotMap.ROBOT_LOOP);
        return loop.getU(0);
    }

    public double getSetpoint() {
        return loop.getNextR(0);
    }

    public double getError() {
        return loop.getError(0);
    }
}
