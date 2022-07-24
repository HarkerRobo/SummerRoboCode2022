package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;

public class ColorSensor {
  private DigitalInput A;
  private DigitalInput B;
  private DigitalInput proximity;
  private boolean isRed;
  private boolean isFunctioning;

  public ColorSensor(int A, int B, int proximity) {
    this.A = new DigitalInput(A);
    this.B = new DigitalInput(B);
    this.proximity = new DigitalInput(proximity);
    isFunctioning = true;
  }

  public boolean inProximity() {
    return !proximity.get();
  }

  public boolean isFunctioning() {
    return isFunctioning;
  }

  public boolean isRed() {
    return isRed;
  }

  public boolean getA() {
    return A.get();
  }

  public boolean getB() {
    return B.get();
  }

  public void set(boolean functioning, boolean red) {
    isFunctioning = functioning;
    isRed = red;
  }
}
