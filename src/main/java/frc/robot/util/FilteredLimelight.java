package frc.robot.util;
import edu.wpi.first.math.filter.LinearFilter;
import harkerrobolib.util.Limelight;

public class FilteredLimelight{
    private static Limelight limelight;
    private static LinearFilter txFilter = LinearFilter.movingAverage(8);
    private static LinearFilter tyFilter = LinearFilter.movingAverage(8);
    private static double currentTx;
    private static double currentTy;

    public static void update() {
        currentTx = txFilter.calculate(Limelight.table.getEntry(Limelight.TX_KEY).getDouble(0.0));
        currentTy = tyFilter.calculate(Limelight.table.getEntry(Limelight.TY_KEY).getDouble(0.0));
    }

    public static double getTx() {
        return currentTx;
    }

    public static double getTy() {
        return currentTy;
    }

    public static Limelight getLimelight() {
        return limelight;
    }
    
}
