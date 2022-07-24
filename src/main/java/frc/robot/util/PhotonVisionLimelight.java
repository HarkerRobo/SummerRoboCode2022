package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotMap;
import org.photonvision.PhotonCamera;

public class PhotonVisionLimelight {
  private final PhotonCamera limelight = new PhotonCamera(RobotMap.LIMELIGHT_NAME);

  public static Translation2d robotToTarget() {
    return null;
  }

  public static double lastMeasurementLatency() {
    return 0.0;
  }
}
