//Currently it does not seem like this command is needed, but I'm not deleting it incase it has an actual use in the future


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
    if (this.speed < 0 && climber.leftHitLimitSwitch()) { //This only works for if the climber is at its lowest, remember to add another condition for if it is at its highest
        climber.resetLeftClimberEncoder();
        climber.leftClimbspeed(0);
    } else {
        climber.leftClimbspeed(speed);
    }

    if (this.speed < 0 && climber.rightHitLimitSwitch()) {
        climber.resetRightClimberEncoder();
        climber.rightClimbspeed(0);
    } else {
        climber.rightClimbspeed(speed);
    }
  }

@Override
public void end(boolean interuppted) {
    climber.leftClimbspeed(0);
    climber.rightClimbspeed(0);
}

@Override
public boolean isFinished() {
    return false;
  }
}