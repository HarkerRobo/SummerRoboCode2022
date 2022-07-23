package frc.robot.util.loop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import frc.robot.RobotMap;

public class VelocityControlLoop extends LinearSystemControlLoop<N1, N1, N1> {

    protected VelocityControlLoop(double kV, double kA, double kS, double modelStdDev, double encoderStdDev, double maxError, double maxControlEffort, double latencyCompensation, double dtSeconds) {
        super(VecBuilder.fill(-kV/kA), 
              VecBuilder.fill(1.0/kA), 
              VecBuilder.fill(1.0), 
              VecBuilder.fill(0.0), 
              Nat.N1(), Nat.N1(), 
              VecBuilder.fill(maxError), 
              VecBuilder.fill(maxControlEffort), 
              VecBuilder.fill(modelStdDev), 
              VecBuilder.fill(encoderStdDev), 
              latencyCompensation, (u) -> {
                double input = u.get(0,0);
                if (Math.abs(input) > 0.005) input += kS;
                input = MathUtil.clamp(input, -maxControlEffort, maxControlEffort);
                return VecBuilder.fill(input);
              }, dtSeconds);
    }

    public double setReferenceAndPredict(double setpoint, double systemOutput) {
        return setReferenceAndPredict(VecBuilder.fill(setpoint), VecBuilder.fill(systemOutput)).get(0,0);
    }

    public double correctAndPredict(double systemOutput) {
        return correctAndPredict(VecBuilder.fill(systemOutput)).get(0,0);
    }
    
    public void reset(double initVelocity) {
        super.reset(VecBuilder.fill(initVelocity));
    }

    public double getSetpoint() {
        return super.getNextR().get(0, 0);
    }

    public double getFilteredVelocity() {
        return super.getXHat().get(0,0);
    }

    public double getVelocityError() {
        return super.getError().get(0,0);
    }

    public static class VelocityControlLoopBuilder {
        protected double kS;
        protected double kV;
        protected double kA;
        protected double modelSTDdev;
        protected double measSTDdev;
        protected double maxError;
        protected double maxControlEffort = RobotMap.MAX_MOTOR_VOLTAGE;
        protected double latencyCompensation = 0.0;
        protected double dtSeconds = RobotMap.ROBOT_LOOP;

        public VelocityControlLoopBuilder motorConstants(double kS, double kA, double kV) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            return this;
        }

        public VelocityControlLoopBuilder standardDeviations(double model, double meas) {
            this.modelSTDdev = model;
            this.measSTDdev = meas;
            return this;
        }
        
        public VelocityControlLoopBuilder maxError(double maxError) {
            this.maxError = maxError;
            return this;
        }

        public VelocityControlLoopBuilder maxControlEffort(double maxControlEffort) {
            this.maxControlEffort = maxControlEffort;
            return this;
        }

        public VelocityControlLoopBuilder latency(double latency) {
            this.latencyCompensation = latency;
            return this;
        }

        public VelocityControlLoopBuilder dtSeconds(double dtSeconds) {
            this.dtSeconds = dtSeconds;
            return this;
        }

        public VelocityControlLoop buildVelocityControlLoop() {
            return new VelocityControlLoop(kV, kA, kS, modelSTDdev, measSTDdev, maxError, maxControlEffort, latencyCompensation, dtSeconds);
        }
    }
}
