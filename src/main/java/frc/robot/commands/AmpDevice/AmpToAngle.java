package frc.robot.commands.AmpDevice;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpDevice.AmpDeviceSubsystem;

public class AmpToAngle extends Command{
    private final AmpDeviceSubsystem amp;
    private double angle;

public AmpToAngle(AmpDeviceSubsystem amp, double angle) {
    this.amp = amp;
    this.angle = angle;
    addRequirements(amp);
}

 @Override
public void initialize() {}
   
@Override
public void execute() {
    amp.PivotAmpToAngle(this.angle);
}

@Override
public void end(boolean interuppted) {
    amp.AmpDeviceOut(0);
}

@Override
public boolean isFinished() {
    return true;
    // Math.abs(amp.getDeviceAngle() - this.angle) < 0.5;
}

}
