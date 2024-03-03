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
        if (this.speed > 0 && !intake.hasNote()) {
            intake.setSpeed(speed);
            intake.indexerSpeed(speed);

        } else {
            intake.setSpeed(0);
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
