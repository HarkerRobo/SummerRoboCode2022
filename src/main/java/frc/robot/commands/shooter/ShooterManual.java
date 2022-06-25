package frc.robot.commands.shooter;

import frc.robot.OI;
import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;

public class ShooterManual extends IndefiniteCommand{
    public ShooterManual() {
        addRequirements(Shooter.getInstance());
    }

    public void execute() {
        Shooter.getInstance().set(calculateShooterSpeed());
    }

    public double calculateShooterSpeed() {
        if(OI.getInstance().getDriverGamepad().getButtonYState())
            return 30;
        return 0;
    }
}
