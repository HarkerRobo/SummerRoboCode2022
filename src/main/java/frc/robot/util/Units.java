package frc.robot.util;

public class Units {
    public static final double ENCODER_TICKS_TO_DEGREES = 360.0/2048.0;
    public static final double DEGREES_TO_ENCODER_TICKS = 1.0/ENCODER_TICKS_TO_DEGREES;
    public static final double FALCON_ENCODER_TICKS = 2048.0;
    public static final double MAG_CODER_ENCODER_TICKS = 4096.0;
    public static final double FOUR_INCH_WHEEL_ROT_TO_METER = 0.1016 * Math.PI;
    public static final double TALON_VELOCITY_TO_ROT_PER_SECOND = 10 / FALCON_ENCODER_TICKS;
}
