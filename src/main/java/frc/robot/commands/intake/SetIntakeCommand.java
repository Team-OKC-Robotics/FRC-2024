package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;

public class SetIntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private double speed;
    
    private PivotSubsystem pivot;

    public SetIntakeCommand(IntakeSubsystem intake, double speed) {
        this.intake = intake;
        this.speed = speed;

        addRequirements(intake);
    }

 @Override
 public void initialize() {}

    @Override
    public void execute() {

        if (this.speed > 0) {   //if you actually set a speed
            
            if (!intake.hasNote()) { //and the intake does not have the note
                intake.setSpeed(speed);     //then run the indexer and intake 
                intake.indexerSpeed(speed);
            } else if (intake.secondSwitchHit()) { //if the note is far up enough in the intake

                if (pivot.getPivotAngle() < 42) { //and the pivot ange is low enough
                    intake.setSpeed(speed / 2);     //then slow down the speed
                    intake.indexerSpeed(speed / 2);
                } else {
                    intake.setSpeed(0);     //set the speed to 0
                    intake.indexerSpeed(0);                    
                }

            } else {    //if no switch is hit
                intake.setSpeed(0);     //set the speed to 0
                intake.indexerSpeed(0);
            }

        } else {    //if you didn't set any speed
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
