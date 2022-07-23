package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class SetClimberPos extends CommandBase{
    private double position;
    
    private static final double POS_THRESHOLD = 50; //TODO

    public SetClimberPos(double pos) {
        addRequirements(Climber.getInstance());
        position = pos;
    }
    
    public void initialize() {
        Climber.getInstance().getLeftControlLoop().reset(Climber.getInstance().getLeftClimberPos(), Climber.getInstance().getLeftClimberVel());
        Climber.getInstance().getRightControlLoop().reset(Climber.getInstance().getRightClimberPos(), Climber.getInstance().getRightClimberVel());
    }
    public void execute() {
        Climber.getInstance().setBothClimberPos(position);
    }

    public boolean isFinished() {
        return Climber.getInstance().getLeftControlLoop().getPositionError() < POS_THRESHOLD
        && Climber.getInstance().getRightControlLoop().getPositionError() < POS_THRESHOLD; 
    }

    public void end(boolean interrupted) {
        Climber.getInstance().turnOffMotors();
    }
}
