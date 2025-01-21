package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.*;

public class PivotToAngle extends Command {
    private final PivotSubsystem pivot;
    private double angle;

    public PivotToAngle(PivotSubsystem pivot, double angle) {
        this.pivot = pivot;
        this.angle = angle;
        addRequirements(pivot);
    }

    public PivotToAngle(PivotSubsystem pivot, PivotSubsystem.PivotLocations location) {
        this.pivot = pivot;
        this.angle = location.getCommandedAngle();
        addRequirements(pivot);
    }

    @Override

    public void initialize() {
        pivot.setTargetPivotAngle(angle);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interuppted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
