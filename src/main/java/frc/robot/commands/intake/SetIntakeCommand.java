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
        intake.setSpeed(1);
    }

@Override
public void end(boolean interuppted) {
    intake.setSpeed(0);
}

    @Override
    public boolean isFinished() {
        return false;
    }
}
