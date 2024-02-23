package frc.robot.commands.climber;
import frc.robot.subsystems.climber.ClimberSubsystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberCommand extends Command{
   
    private final ClimberSubsystem climber;
    private double speed;
    
    

public ClimberCommand(double speed, ClimberSubsystem climber) {
    this.climber = climber;
    this.speed = speed; 
    addRequirements(climber);
}

@Override
public void initialize() {
    
}

@Override 
public void execute() {
    climber.climbspeed(1);

}

@Override
public void end(boolean interuppted) {
    climber.climbspeed(0);
}

@Override
public boolean isFinished() {
    return false;
    }
}
