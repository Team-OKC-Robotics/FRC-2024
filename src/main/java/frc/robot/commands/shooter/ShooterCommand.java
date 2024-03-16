// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterCommand extends Command {
    private final ShooterSubsystem shooter;
    
    private double speed;
    private double power;

public ShooterCommand(ShooterSubsystem shooter) {
    this.shooter = shooter;
    

    addRequirements(shooter);
}
@Override
 public void initialize() {}

@Override
public void execute() {
    shooter.setShooterSpeed(1.0);

    shooter.setIntakeSpeed(1.0);
   // shooter.indexerSpeed(1);
    
    
}

@Override
public void end(boolean interuppted) {
    shooter.stopShooter();
    shooter.stopIntake();
  //  shooter.indexerSpeed(0);
}

@Override
public boolean isFinished() {
    return false;
    }   
}
