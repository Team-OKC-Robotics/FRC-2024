// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.utils.LerpedLUT;

public class AutoAimInAuto extends Command {
  /** Creates a new AutoAim. */
  private final Vision vision;
  private final PivotSubsystem pivot;
  private int targetAprilTag = 4;
  private Distance targetOffset = Feet.of(3.9);

  LerpedLUT angleLUT = new LerpedLUT();

  public AutoAimInAuto(Vision vision, PivotSubsystem pivot) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(pivot);

    this.vision = vision;
    this.pivot = pivot;

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      this.targetAprilTag = 7;
    }

    angleLUT.addEntry(-100, 60);
    angleLUT.addEntry(0, 58); // distance in feet, angle in degrees
    angleLUT.addEntry(1.8, 44.5);
    angleLUT.addEntry(2.17, 43);
    angleLUT.addEntry(3.37, 38);
    angleLUT.addEntry(3.9, 34);
    angleLUT.addEntry(4.33, 33.7);
    angleLUT.addEntry(5.0, 31.5);
    angleLUT.addEntry(5.33, 32.8);
    angleLUT.addEntry(5.5, 30.6);
    angleLUT.addEntry(6.33, 26);
  }

  double tagHeight = 57.13;
  double cameraHeight = 25;
  double distanceThreshold = 0; // placeholder
  double cameraAngle = 30; // placeholder
  double angleThreshold = 0; // placeholder

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<Double> targetDistanceOptional = vision.getDistanceFromAprilTag(targetAprilTag);
    Distance targetDistance = Distance.ofBaseUnits(100, Feet);
    if (targetDistanceOptional.isPresent()) {
      targetDistance = Meters.of(targetDistanceOptional.get()).minus(targetOffset);
    }
    pivot.setTargetPivotAngle(angleLUT.getAngleFromDistance(targetDistance));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // pivot.PivotIt(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
