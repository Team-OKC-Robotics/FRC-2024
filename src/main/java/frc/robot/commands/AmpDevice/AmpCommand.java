package frc.robot.commands.AmpDevice;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.*;

public class AmpCommand extends Command {
    private final PivotSubsystem amp;
    private double angle;

public AmpCommand(PivotSubsystem amp, double angle) {
    this.amp = amp;
    this.angle = angle;
    addRequirements(amp);
}

@Override
 public void initialize() {}

@Override
public void execute() {
    amp.desireAmpEngaged();
}

@Override
public void end(boolean interuppted) {
    amp.desireAmpIn();
}

@Override
public boolean isFinished() {
    return false;
}
}


