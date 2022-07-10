package frc.robot.commands.shooter;

import frc.robot.OI;
import frc.robot.subsystems.Shooter;
import frc.robot.util.InterpolatingTreeMap;
import harkerrobolib.commands.IndefiniteCommand;

public class ShooterManual extends IndefiniteCommand{
    private InterpolatingTreeMap shooterVals;
    public ShooterManual() {
        addRequirements(Shooter.getInstance());
        shooterVals = new InterpolatingTreeMap();
    }

    public void execute() {
        Shooter.getInstance().set(calculateShooterSpeed());
    }

    public double calculateShooterSpeed() {
        if(OI.getInstance().getDriverGamepad().getButtonYState())
            return 5;
        return 0;
    }

    public void end(boolean interrupted) {
        Shooter.getInstance().set(0);
    }
}
