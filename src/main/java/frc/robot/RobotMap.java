package frc.robot;

public final class RobotMap {
    public static final String CANBUS = "BINGCHILLING";
    public static final double ROBOT_LOOP = 0.02;
    public static final double MAX_MOTOR_VOLTAGE = 12;

    public static final int DEFAULT_LOOP_ID = 0;
    public static final int DEFAULT_SLOT_ID = 0;

    public static final int[] TRANSLATION_IDS = {13, 4, 2, 6}; //FL, FR, BL, BR
    public static final int[] ROTATION_IDS = {1, 5, 3, 7};
    public static final int[] CANCODER_IDS = {0, 2, 1, 3};
    public static final int PIGEON_ID = 0;

    public static final int INTAKE_FORWARD = 7;
    public static final int INTAKE_BACKWARD = 3;
    public static final int INTAKE_MOTOR = 8;

    public static final int INDEXER_TOP = 10;
    public static final int INDEXER_BOTTOM = 9;

    public static final int HOOD_ID = 14;

    public static final int TOP_PROXIMITY = 7;
    public static final int BOTTOM_PROXIMITY = 6;

    public static final int SHOOTER_MASTER = 11;
    public static final int SHOOTER_FOLLOWER = 12;

    public static final int OPERATOR_ID = 1;
    public static final int DRIVER_ID = 0;


}
