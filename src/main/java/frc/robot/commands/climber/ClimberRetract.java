package frc.robot.commands.climber;
import frc.robot.subsystems.climber.ClimberSubsystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberRetract extends Command {
   
    private final ClimberSubsystem climber;
    private double speed;
    
    

public ClimberRetract(ClimberSubsystem climber, double speed) {
    this.climber = climber;
    this.speed = speed; 
    addRequirements(climber);
}

@Override
public void initialize() {}

@Override 
public void execute() {
    if (this.speed < 0 && climber.hasClimbed()) {
        climber.climbretractspeed(speed);
}
else {
    climber.climbretractspeed(0);
    }
}

@Override
public void end(boolean interuppted) {
    climber.climbretractspeed(0);
}

@Override
public boolean isFinished() {
    return climber.hasClimbed();
    }
}