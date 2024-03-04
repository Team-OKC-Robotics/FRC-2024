package frc.robot.commands.shooter;


import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootWaitAuto extends Command {
     
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;

    
    private double speed;
    private double power;
    private boolean hasNoteLeft = false;

public ShootWaitAuto(ShooterSubsystem shooter, IntakeSubsystem intake, double power) {
    this.shooter = shooter;
    this.intake = intake;
    this.power = power;

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
    shooter.shootSpeed(5000);
    if (shooter.getMinVelocity() > 4700) {
        intake.SetIntake(1);
    }

   // shooter.indexerSpeed(1);
    
    
}

@Override
public void end(boolean interuppted) {
   // shooter.stopShooter();
    intake.SetIntake(0);
  //  shooter.indexerSpeed(0);
}

@Override
public boolean isFinished() {
    return shooter.getMinVelocity() < 2000 && hasNoteLeft;
    }   
}



