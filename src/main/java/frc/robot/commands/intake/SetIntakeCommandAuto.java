package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class SetIntakeCommandAuto extends Command {
    private final IntakeSubsystem intake;
    private double speed;

    public SetIntakeCommandAuto(IntakeSubsystem intake, double speed) {
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
            } else if (/* 2nd Switch Hit & Ange < 42*/) { //if the note is far up enough in the intake
                intake.setSpeed(speed / 2);     //then slow down the speed
                intake.indexerSpeed(speed / 2);
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
 //   intake.setSpeed(0);
 //   intake.indexerSpeed(0);
}

    @Override
    public boolean isFinished() {
        return intake.hasNote();
    }
}
