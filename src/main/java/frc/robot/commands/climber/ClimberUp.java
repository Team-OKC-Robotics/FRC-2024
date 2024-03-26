package frc.robot.commands.climber;
import frc.robot.subsystems.climber.ClimberSubsystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberUp extends Command{
    private final ClimberSubsystem climber;
    private double speed;
    
    

public ClimberUp(ClimberSubsystem climber, double speed) {
    this.climber = climber;
    this.speed = speed; 
    addRequirements(climber);
}

@Override
public void initialize() {}

@Override 
public void execute() {
    if (this.speed < 0 && !climber.hasClimbed()) {
        climber.climbupspeed(speed);
}
else {
    climber.climbupspeed(0);
    }
}

@Override
public void end(boolean interuppted) {
    climber.climbupspeed(0);
}

@Override
public boolean isFinished() {
    return climber.hasClimbed();
    }
}

