package frc.robot.commands.pivot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.*;

public class SetPivotCommand extends Command {
    private final PivotSubsystem pivot;
    private double power;

public SetPivotCommand(PivotSubsystem pivot, double power) {
    this.pivot = pivot;
    this.power = power;
    addRequirements(pivot);
}

@Override
 public void initialize() {}

@Override
public void execute() {
    pivot.PivotIt(0.1);
}

@Override
public void end(boolean interuppted) {
    pivot.PivotIt(0);
}

@Override
public boolean isFinished() {
    return false;
}
}

