package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;

public class ColorSensor {
  private DigitalInput A;
  private DigitalInput B;
  private DigitalInput proximity;
  private boolean isRed;

  public ColorSensor(int A, int B, int proximity) {
    this.A = new DigitalInput(A);
    this.B = new DigitalInput(B);
    this.proximity = new DigitalInput(proximity);
  }

  public boolean inProximity() {
    return !proximity.get();
  }

  public boolean isFunctioning() {
    return !(getA() && getB() || !getA() && !getB());
  }

  public boolean isRightColor() {
    if (isRed) return getA() && !getB();
    return !getA() && getB();
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

  public void setColor(boolean red) {
    isRed = red;
  }
}
