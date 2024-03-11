package frc.robot.commands.climber;
import frc.robot.subsystems.climber.ClimberSubsystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberCommand extends Command {
   
    private final ClimberSubsystem climber;
    private double speed;
    
    

public ClimberCommand(ClimberSubsystem climber, double speed) {
    this.climber = climber;
    this.speed = speed; 
    addRequirements(climber);
}

@Override
public void initialize() {}

@Override 
public void execute() {
    if (this.speed > 0 && !climber.hasClimbed()) {
        climber.climbspeed(speed);
}
else {
    climber.climbspeed(0);
    }
}

@Override
public void end(boolean interuppted) {
    climber.climbspeed(0);
}

@Override
public boolean isFinished() {
    return climber.hasClimbed();
    }
}