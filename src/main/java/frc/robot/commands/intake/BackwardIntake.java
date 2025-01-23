package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class BackwardIntake extends Command {
    private final IntakeSubsystem intake;

    public BackwardIntake(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setStateOuttake();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interuppted) {
        intake.setStateHold();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
