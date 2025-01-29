package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class SetIntakeCommand extends Command {
    private final IntakeSubsystem intake;
    public SetIntakeCommand(IntakeSubsystem intake, double speed) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setStateIntake();
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
        return intake.hasNote(); // the command is finished when the intake has the note
    }
}
