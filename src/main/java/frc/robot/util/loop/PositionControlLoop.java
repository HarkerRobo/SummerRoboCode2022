package frc.robot.util.loop;

import java.util.function.Function;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class PositionControlLoop extends LinearSystemControlLoop<N2, N1, N1> {
    protected double maxControlEffort;

    protected PositionControlLoop(Matrix<N2, N2> A, Matrix<N2, N1> B, Matrix<N1, N2> C, Matrix<N1, N1> D, Matrix<N2, N1> modelSTDDev, 
                double measPositionStdDev, Vector<N2> maxError, double maxControlEffort, 
                double latencyCompensation, Function<Matrix<N1, N1>, Matrix<N1, N1>> clampFunction, double dtSeconds) {
        super(A, B, C, D, Nat.N2(), Nat.N1(), 
            maxError, VecBuilder.fill(maxControlEffort), modelSTDDev, VecBuilder.fill(measPositionStdDev), 
            latencyCompensation, clampFunction, dtSeconds);
    }

    public double setReferenceAndPredict(double setpointPos, double setpointVel, double systemOutput) {
        return setReferenceAndPredict(VecBuilder.fill(setpointPos, setpointVel), VecBuilder.fill(systemOutput)).get(0,0);
    }

    public double correctAndPredict(double systemOutput) {
        return correctAndPredict(VecBuilder.fill(systemOutput)).get(0,0);
    }
    
    public void reset(double initPosition, double initVelocity) {
        super.reset(VecBuilder.fill(initPosition, initVelocity));
    }

    public void setNextSetpoint(double position, double velocity) {
        setNextR(VecBuilder.fill(position, velocity));
    }

    public double getSetpointPosition() {
        return super.getNextR().get(0, 0);
    }

    public double getSetpointVelocity() {
        return super.getNextR().get(1, 0);
    }

    public double getFilteredPosition() {
        return super.getXHat().get(0,0);
    }

    public double getFilteredVelocity() {
        return super.getXHat().get(1,0);
    }

    public double getPositionError() {
        return super.getError().get(0, 0);
    }

    public static class PositionControlLoopBuilder extends VelocityControlLoop.VelocityControlLoopBuilder{
        private Matrix<N2, N2> A;
        private Matrix<N2, N1> B;
        private Matrix<N1, N2> C;
        private Matrix<N1, N1> D;
        private Vector<N2> modelSTDdev;
        private Vector<N2> maxError;
        private double kG;

        public PositionControlLoopBuilder motorConstants(double kS, double kA, double kV) {
            return motorConstants(kS, kA, kV, 0.0);
        }

        public PositionControlLoopBuilder motorConstants(double kS, double kA, double kV, double kG) {
            this.kS = kS;
            this.kG = kG;
            A = Matrix.mat(Nat.N2(), Nat.N2()).fill(0.0, 1.0, 0.0, -kV/kA);
            B = VecBuilder.fill(0.0, 1.0 / kA);
            C = Matrix.mat(Nat.N1(), Nat.N2()).fill(1.0, 0.0);
            D = VecBuilder.fill(0.0);
            return this;
        }

        public PositionControlLoopBuilder stateMatrices(Matrix<N2, N2> A, Matrix<N2, N1> B, Matrix<N1, N2> C, Matrix<N1, N1> D) {
            this.A = A;
            this.B = B;
            this.C = C;
            this.D = D;
            return this;
        }

        public PositionControlLoopBuilder standardDeviations(double modelPos, double modelVel, double meas) {
            this.modelSTDdev = VecBuilder.fill(modelPos, modelVel);
            this.measSTDdev = meas;
            return this;
        }
        
        public PositionControlLoopBuilder maxError(double maxPosError, double maxVelError) {
            this.maxError = VecBuilder.fill(maxPosError, maxVelError);
            return this;
        }

        public PositionControlLoopBuilder maxControlEffort(double maxControlEffort) {
            this.maxControlEffort = maxControlEffort;
            return this;
        }

        public PositionControlLoopBuilder latency(double latency) {
            this.latencyCompensation = latency;
            return this;
        }

        public PositionControlLoopBuilder dtSeconds(double dtSeconds) {
            this.dtSeconds = dtSeconds;
            return this;
        }

        public PositionControlLoop buildPositionControlLoop() {
            Function<Matrix<N1, N1>, Matrix<N1, N1>> clampFunction;
            if(kS > 0.005)
                clampFunction = (u) -> {
                    double input = u.get(0,0);
                    if (Math.abs(input) > 0.005) input += kS;
                    input = MathUtil.clamp(input, -maxControlEffort, maxControlEffort);
                    return VecBuilder.fill(input);
                  };
            else 
                  clampFunction = (u) -> VecBuilder.fill(MathUtil.clamp(u.get(0,0), -maxControlEffort, maxControlEffort));
            return new PositionControlLoop(A, B, C, D, modelSTDdev, measSTDdev, maxError, maxControlEffort, latencyCompensation, clampFunction, dtSeconds);
        }

        public PositionControlLoop buildElevatorControlLoop() {
            return new PositionControlLoop(A, B, C, D, modelSTDdev, measSTDdev, maxError, maxControlEffort, latencyCompensation, (u) -> {
                double input = u.get(0,0);
                if (Math.abs(input) > 0.005) input += kS;
                input += kG;
                input = MathUtil.clamp(input, -maxControlEffort, maxControlEffort);
                return VecBuilder.fill(input);
              }, dtSeconds);
        }

        public PositionControlLoop buildArmControlLoop() {
            PositionControlLoop loop = new PositionControlLoop(A, B, C, D, modelSTDdev, measSTDdev, maxError, maxControlEffort, latencyCompensation, (u)->u, dtSeconds);
            loop.setClampFunction((u) -> {
                double input = u.get(0,0);
                if (Math.abs(input) > 0.005) input += kS;
                input += kG * Math.cos(Math.toRadians(loop.getFilteredPosition()));
                input = MathUtil.clamp(input, -maxControlEffort, maxControlEffort);
                return VecBuilder.fill(input);
            });
            return loop;
        }
    }
}
