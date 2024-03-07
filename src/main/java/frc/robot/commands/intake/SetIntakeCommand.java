package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class SetIntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private double speed;

    public SetIntakeCommand(IntakeSubsystem intake, double speed) {
        this.intake = intake;
        this.speed = speed;

        addRequirements(intake);
    }

 @Override
 public void initialize() {}

    @Override
    public void execute() {
        if (this.speed > 0 && !intake.hasNote()) { //if the intake does not have the note
            intake.setSpeed(speed);     //then run the indexer and intake 
            intake.indexerSpeed(speed);

        } else {    //otherwise
            intake.setSpeed(0);       //the intake and indexer should be off
            intake.indexerSpeed(0);
        }
    }

@Override
public void end(boolean interuppted) {
    intake.setSpeed(0);
    intake.indexerSpeed(0);
}

    @Override
    public boolean isFinished() {
        return intake.hasNote();  //the command is finished when the intake has the note
    }
}
