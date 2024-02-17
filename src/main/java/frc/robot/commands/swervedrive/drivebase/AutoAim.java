// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoAim extends Command {
  /** Creates a new AutoAim. */
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;

  public AutoAim(SwerveSubsystem swerve, VisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(swerve, vision);

    this.swerve = swerve;
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonTrackedTarget target = vision.getTargetWithID(4);
    vision.getTargetWithID(4);

    if (target == null) {
      swerve.drive(new Translation2d(0,0), 0, false);
    } else {
      double yaw = vision.getYaw(4);
      swerve.drive(new Translation2d(0,0), -0.1 * yaw, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}