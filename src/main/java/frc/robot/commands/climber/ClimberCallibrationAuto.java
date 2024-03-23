package frc.robot.commands.climber;
import frc.robot.subsystems.climber.ClimberSubsystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberCallibrationAuto extends Command {
   
    private final ClimberSubsystem climber;
    private double speed;
    
    

public ClimberCallibrationAuto(ClimberSubsystem climber, double speed) {
    this.climber = climber;
    this.speed = speed; 
    addRequirements(climber);
}

@Override
public void initialize() {}

@Override 
public void execute() {
    if (this.speed > 0 && !climber.leftPositionAtLowest()) {
        climber.leftClimbspeed(speed);
    } else {
    climber.leftClimbspeed(0);
    }

    if (this.speed > 0 && !climber.rightPositionAtLowest()) {
        climber.rightClimbspeed(speed);
    } else {
    climber.rightClimbspeed(0);
    }
  }

@Override
public void end(boolean interuppted) {
    climber.leftClimbspeed(0);
    climber.rightClimbspeed(0);
}

@Override
public boolean isFinished() {
    if(climber.rightHasClimbed() && climber.leftHasClimbed()) {
      return true;
    } else {
        return false;
    }
  }
}