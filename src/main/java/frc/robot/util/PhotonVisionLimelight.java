package frc.robot.util;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotMap;

public class PhotonVisionLimelight {
    private final PhotonCamera limelight = new PhotonCamera(RobotMap.LIMELIGHT_NAME);


    public static Translation2d robotToTarget() {
        return null;
    }

    public static double lastMeasurementLatency() {
        return 0.0;
    }
}
