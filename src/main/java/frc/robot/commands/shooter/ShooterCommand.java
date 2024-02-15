package frc.robot.commands.shooter;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterCommand extends Command {
    private final ShooterSubsystem shooter;
    private double power;

public ShooterCommand(ShooterSubsystem shooter, double power) {
    this.shooter = shooter;
    this.power = power;

    addRequirements(shooter);
}
@Override
 public void initialize() {}

@Override
public void execute() {
    shooter.shootSpeed(1);
    
}

@Override
public void end(boolean interuppted) {
    shooter.shootSpeed(0);
}

@Override
public boolean isFinished() {
    return false;
    }   
}
