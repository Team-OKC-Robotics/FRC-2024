package frc.robot.commands.AmpDevice;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpDevice.*;

public class AmpCommand extends Command {
    private final AmpDeviceSubsystem amp;
    private double power;

public AmpCommand(AmpDeviceSubsystem amp, double power) {
    this.amp = amp;
    this.power = power;
    addRequirements(amp);
}

@Override
 public void initialize() {}

@Override
public void execute() {
    amp.AmpDeviceOut(0.3);
}

@Override
public void end(boolean interuppted) {
    amp.AmpDeviceOut(0);
}

@Override
public boolean isFinished() {
    return false;
}
}


