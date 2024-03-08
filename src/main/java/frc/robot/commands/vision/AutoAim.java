// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.vision;

import frc.robot.commands.swervedrive.drivebase.*;

import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoAim extends Command {
  /** Creates a new AutoAim. */
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private int targetAprilTag = 4;

  public AutoAim(SwerveSubsystem swerve, VisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(swerve, vision);

    this.swerve = swerve;
    this.vision = vision;

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance.get() == DriverStation.Alliance.Blue) {
      this.targetAprilTag = 7;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonTrackedTarget target = vision.getTargetWithID(targetAprilTag);
    // vision.getTargetWithID(4);

    swerve.getTargetSpeeds(0, 0, swerve.getHeading().getSin(), swerve.getHeading().getCos());

    if (target == null) {
      swerve.drive(new Translation2d(0,0), 0, true);
    } else {
      double yaw = vision.getYaw(targetAprilTag);
      swerve.drive(new Translation2d(0,0), -0.1 * yaw, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (vision.getTargetWithID(targetAprilTag) == null) {
      return false;
    } else {
      return Math.abs(vision.getTargetWithID(targetAprilTag).getYaw()) < 1;
  }
}
}
