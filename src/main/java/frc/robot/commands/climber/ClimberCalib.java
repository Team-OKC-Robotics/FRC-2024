package frc.robot.commands.climber;
import frc.robot.subsystems.climber.ClimberSubsystem;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.*;

public class ClimberCalib extends Command{
    private final ClimberSubsystem climber;
    private double speed;

public ClimberCalib(ClimberSubsystem climber, double speed) {
    this.climber = climber;
    this.speed = speed;
    addRequirements(climber);
}

@Override
public void initialize() {}

@Override 
public void execute(){
    if (this.speed < 0 && climber.hasleftHit()) {
        climber.resetleftencoder();
        climber.leftclimbspeed(0);
    } else {
        climber.leftclimbspeed(speed);
    }
    if (this.speed < 0 && climber.hasrightHit()) {
        climber.resetrightencoder();
        climber.rightclimbspeed(0);
    } else {
        climber.rightclimbspeed(speed);
    }
  }   

@Override 
public void end(boolean interuppted) {
    climber.leftclimbspeed(0);
    climber.rightclimbspeed(0);
}
}
