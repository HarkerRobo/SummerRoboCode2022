package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
    public static final double FIELD_LENGTH = Units.inchesToMeters(54.0 * 12.0);
    public static final double FIELD_WIDTH = FIELD_LENGTH / 2.0;
    public static final double HUB_RADIUS = Units.inchesToMeters(427.0/16.0);
    public static final Translation2d HUB_LOCATION = new Translation2d(FIELD_LENGTH/2.0, FIELD_WIDTH/2.0);
}
