package frc.robot.commands.climber;
import frc.robot.subsystems.climber.ClimberSubsystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimbUp extends Command{
   
    private final ClimberSubsystem climber;
    private double speed;
    
    

public ClimbUp(double speed, ClimberSubsystem climber) {
    this.climber = climber;
    this.speed = speed; 
    addRequirements(climber);
}

@Override
public void initialize() {
    climber.setspeed(speed);
   
    }
@Override 
public void execute() {}

@Override
public void end(boolean interuppted) {
    climber.setspeed(0);
}

@Override
public boolean isFinished() {
    return false;
    }
}
