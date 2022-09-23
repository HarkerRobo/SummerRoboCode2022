package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotMap;

import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionLimelight {
  private static final PhotonCamera LIMELIGHT = new PhotonCamera(RobotMap.LIMELIGHT_NAME);
  private static final double PRECISION = 0.05;

  private static final Translation2d cameraToVehicle = new Translation2d(0.32, 0.0);
  private static final double LIMELIGHT_HEIGHT = 0.603284;
  private static final double LIMELIGHT_TO_HUB_HEIGHT = Constants.HUB_HEIGHT - LIMELIGHT_HEIGHT;
  private static final double LIMELIGHT_ANGLE = Math.toRadians(38);
  private static Translation2d robotToHub = null;

  public static void robotToHub() {
    if(!LIMELIGHT.getLatestResult().hasTargets()) return;
    List<Translation2d> points = new ArrayList<>();
    for (PhotonTrackedTarget trackedTarget : LIMELIGHT.getLatestResult().getTargets()) {
      points.add(getRobotToTarget(trackedTarget));
    }
    Translation2d cameraToTarget = fit(points, PRECISION);
    Translation2d targetToVehicle = cameraToTarget.plus(cameraToVehicle);
    robotToHub =  targetToVehicle;
  }

  public static double getDistance() {
    return robotToHub.getNorm();
  }

  public static double getXDistance() {
    return robotToHub.getX();
  }

  public static double lastMeasurementLatency() {
    return LIMELIGHT.getLatestResult().getLatencyMillis() / 1000.0;
  }

  public static Translation2d fit(List<Translation2d> points, double precision) {

    // Find starting point
    double xSum = 0.0;
    double ySum = 0.0;
    for (Translation2d point : points) {
      xSum += point.getX();
      ySum += point.getY();
    }
    Translation2d center =
        new Translation2d(xSum / points.size() + Constants.HUB_RADIUS, ySum / points.size());

    // Iterate to find optimal center
    double shiftDist = Constants.HUB_RADIUS / 2.0;
    double minResidual = calcResidual(points, center);
    while (true) {
      List<Translation2d> translations =
          List.of(
              new Translation2d(shiftDist, 0.0),
              new Translation2d(-shiftDist, 0.0),
              new Translation2d(0.0, shiftDist),
              new Translation2d(0.0, -shiftDist));
      Translation2d bestPoint = center;
      boolean centerIsBest = true;

      // Check all adjacent positions
      for (Translation2d translation : translations) {
        double residual = calcResidual(points, center.plus(translation));
        if (residual < minResidual) {
          bestPoint = center.plus(translation);
          minResidual = residual;
          centerIsBest = false;
          break;
        }
      }

      // Decrease shift, exit, or continue
      if (centerIsBest) {
        shiftDist /= 2.0;
        if (shiftDist < precision) {
          return center;
        }
      } else {
        center = bestPoint;
      }
    }
  }

  private static double calcResidual(List<Translation2d> points, Translation2d center) {
    double residual = 0.0;
    for (Translation2d point : points) {
      double diff = point.getDistance(center) - Constants.HUB_RADIUS;
      residual += diff * diff;
    }
    return residual;
  }

  private static Translation2d getRobotToTarget(PhotonTrackedTarget target) {
    double dy = Math.toRadians(target.getPitch());
    double dx = Math.toRadians(target.getYaw());
    double dist = LIMELIGHT_TO_HUB_HEIGHT / (Math.tan(dy + LIMELIGHT_ANGLE) * Math.cos(dx));
    return new Translation2d(Math.cos(dx) * dist, Math.sin(dx) * dist);
  }
}
