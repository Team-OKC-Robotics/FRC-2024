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
    if (this.speed < 0 && climber.hasleftHit()) { //This only works for if the climber is at its lowest, remember to add another condition for if it is at its highest
        climber.leftclimbspeed(0);
    } else {
        climber.leftclimbspeed(climber.isleftClose() ? 0.7 * speed : 1.0 * speed);
    }

    if (this.speed < 0 && climber.hasrightHit()) {
        climber.rightclimbspeed(0);
    } else {
        climber.rightclimbspeed(climber.isrightClose() ? 0.7 * speed : 1.0 * speed);
    }
  }

@Override
public void end(boolean interuppted) {
    climber.leftclimbspeed(0);
    climber.rightclimbspeed(0);
}

@Override
public boolean isFinished() {
    return false;
  }
}
