package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    public static final boolean[] ROTATION_INVERTS = {false, false, false, false};
    public static final boolean[] DRIVE_INVERTS = {true, true, true, false};

    public static final double[] CANCODER_OFFSETS = {195.468750, 178.330078, 109.951172, 32.255859}
}
