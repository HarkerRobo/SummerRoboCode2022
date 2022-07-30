package frc.robot.util.loop;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import frc.robot.RobotMap;
import java.util.function.Function;

public class LinearSystemControlLoop<States extends Num, Inputs extends Num, Outputs extends Num> {

  private LinearSystemLoop<States, Inputs, Outputs> loop;
  private Function<Matrix<Inputs, N1>, Matrix<Inputs, N1>> clampFunction;
  private double dtSeconds;

  public LinearSystemControlLoop(
      Matrix<States, States> A,
      Matrix<States, Inputs> B,
      Matrix<Outputs, States> C,
      Matrix<Outputs, Inputs> D,
      Nat<States> states,
      Nat<Outputs> outputs,
      Vector<States> qelms,
      Vector<Inputs> relms,
      Matrix<States, N1> stateStdDevs,
      Matrix<Outputs, N1> measurementStdDevs,
      double latencyCompensation,
      Function<Matrix<Inputs, N1>, Matrix<Inputs, N1>> clampFunction,
      double dtSeconds) {

    LinearSystem<States, Inputs, Outputs> plant = new LinearSystem<>(A, B, C, D);
    loop =
        new LinearSystemLoop<>(
            plant,
            new LinearQuadraticRegulator<>(plant, qelms, relms, dtSeconds),
            new KalmanFilter<>(states, outputs, plant, stateStdDevs, measurementStdDevs, dtSeconds),
            RobotMap.MAX_MOTOR_VOLTAGE,
            dtSeconds);
    loop.getController().latencyCompensate(plant, dtSeconds, latencyCompensation);
    this.clampFunction = clampFunction;
    this.dtSeconds = dtSeconds;
  }

  protected Matrix<Inputs, N1> setReferenceAndPredict(
      Matrix<States, N1> setpoint, Matrix<Outputs, N1> systemOutput) {
    loop.setNextR(setpoint);
    return correctAndPredict(systemOutput);
  }

  protected Matrix<Inputs, N1> correctAndPredict(Matrix<Outputs, N1> systemOutput) {
    loop.correct(systemOutput);
    loop.predict(dtSeconds);
    return getPlantInput();
  }

  protected Matrix<Inputs, N1> getPlantInput() {
    return clampFunction.apply(loop.getU());
  }

  protected void reset(Matrix<States, N1> initState) {
    loop.reset(initState);
  }

  protected void setNextR(Matrix<States, N1> nextR) {
    loop.setNextR(nextR);
  }

  protected Matrix<States, N1> getNextR() {
    return loop.getNextR();
  }

  protected Matrix<States, N1> getXHat() {
    return loop.getXHat();
  }

  protected Matrix<States, N1> getError() {
    return loop.getError();
  }

  public void setClampFunction(Function<Matrix<Inputs, N1>, Matrix<Inputs, N1>> clampFunction) {
    loop.setClampFunction(clampFunction);
  }

  public LinearQuadraticRegulator<States, Inputs, Outputs> getController() {
    return loop.getController();
  }

  public KalmanFilter<States, Inputs, Outputs> getObserver() {
    return loop.getObserver();
  }

  public LinearPlantInversionFeedforward<States, Inputs, Outputs> getFeedforward() {
    return loop.getFeedforward();
  }
}
