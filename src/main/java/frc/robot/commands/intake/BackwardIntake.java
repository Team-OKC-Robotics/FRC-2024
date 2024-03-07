package frc.robot.commands.intake;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class BackwardIntake extends Command{
    private final IntakeSubsystem intake; 
    private double speed;

    public BackwardIntake(IntakeSubsystem intake, double speed) {
        
        this.intake = intake;
        this.speed = speed;

        addRequirements(intake);
    }

@Override 
public void initialize() {}

@Override
public void execute () {
    intake.setbackSpeed(0.8);
    intake.setIndexerback(0.8);
}

@Override 
public void end (boolean interuppted) {
     
    intake.setSpeed(0);
    intake.indexerSpeed(0);
}


@Override
public boolean isFinished() {
    return false;
}
}

