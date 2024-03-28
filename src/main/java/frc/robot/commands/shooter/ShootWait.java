package frc.robot.commands.shooter;


import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.pivot.*;

public class ShootWait extends Command {
     
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final PivotSubsystem pivot;


    
    private double speed;
    private double RPM;
    private boolean hasNoteLeft = false;

public ShootWait(ShooterSubsystem shooter, IntakeSubsystem intake, PivotSubsystem pivot) {
    this.shooter = shooter;
    this.intake = intake;
    this.pivot = pivot;
    

    addRequirements(shooter, intake);
    
}
@Override
 public void initialize() {
    hasNoteLeft = false;
 }

@Override
public void execute() {
    if (!intake.hasNote()) { 
        hasNoteLeft = true;
        shooter.ShootIt(0);
        return;
    }
     
    if (pivot.IsAmpIn()) {
        RPM = 5000; 
    } else {
        RPM = 1500;
    }
     shooter.shootSpeed(this.RPM);
    if (shooter.getMinVelocity() > (this.RPM * 0.9)) { 
        intake.SetIntake(1);
    }

   // shooter.indexerSpeed(1);
    
    
}

@Override
public void end(boolean interuppted) {
    shooter.stopShooter();
    intake.SetIntake(0);
  //  shooter.indexerSpeed(0);
}

@Override
public boolean isFinished() {
    return shooter.getMinVelocity() < 2000 && hasNoteLeft;
    }   
}


