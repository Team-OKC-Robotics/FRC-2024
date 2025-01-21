// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.LerpedLUT;
import swervelib.SwerveInputStream;

public class AutoAim extends Command {
  /** Creates a new AutoAim. */
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final PivotSubsystem pivot;
  private final LEDSubsystem leds;

  private int targetAprilTag = 4;
  private Distance targetOffset = Feet.of(3.9);

  VisionSubsystem visionSubsystem = new VisionSubsystem();
  LerpedLUT angleLUT = new LerpedLUT();

  private ShuffleboardTab tab = Shuffleboard.getTab("shooter");

  private GenericEntry targetYawEntry = tab.add("Target Yaw", 0.0).getEntry();
  private GenericEntry targetSpeakerIDEntry = tab.add("Target Speaker ID", 0).getEntry();

  private SwerveInputStream swerveInput;

  public AutoAim(SwerveSubsystem swerve, VisionSubsystem vision, PivotSubsystem pivot, LEDSubsystem leds,
      SwerveInputStream swerveInput) {

    addRequirements(swerve, vision, pivot, leds);

    this.swerve = swerve;
    this.vision = vision;
    this.pivot = pivot;
    this.leds = leds;

    this.swerveInput = swerveInput;

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      this.targetAprilTag = 6;
    }

    targetSpeakerIDEntry.setInteger(this.targetAprilTag);

    angleLUT.addEntry(-100, 60);
    angleLUT.addEntry(0, 58); // distance in feet, angle in degrees
    angleLUT.addEntry(2.17, 43);
    angleLUT.addEntry(3.37, 38);
    angleLUT.addEntry(3.9, 35.5);
    angleLUT.addEntry(4.33, 35);
    angleLUT.addEntry(5.0, 31.5);
    angleLUT.addEntry(5.33, 32.8);
    angleLUT.addEntry(5.5, 30.6);
    angleLUT.addEntry(6.33, 29);
  }

  double tagHeight = 57.13;
  double cameraHeight = 25;
  double cameraAngle = 30; // placeholder
  double angleThreshold = 0; // placeholder

  public boolean readyToShoot(Distance distance, double yaw) {
    return distance.in(Feet) < 5.7 && yaw < 3 && pivot.isPivotAtSetpoint();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setLEDState(LEDSubsystem.LEDState.NO_TARGET);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    PhotonTrackedTarget target = vision.getTargetWithID(targetAprilTag);

    double rotationSpeed = 0;
    double targetYaw = 2718;
    if (target != null) {
      targetYaw = target.getYaw();

      if (Math.abs(targetYaw) > 2) {
        rotationSpeed = -0.1 * targetYaw;
      }
    }

    targetYawEntry.setDouble(targetYaw);

    ChassisSpeeds chassisSpeeds = swerveInput.get();
    chassisSpeeds.omegaRadiansPerSecond = rotationSpeed;
    swerve.driveFieldOriented(chassisSpeeds);

    Distance targetDistance = Meters.of(vision.distanceToTarget(target, tagHeight, cameraHeight, cameraAngle)).minus(targetOffset);
    pivot.setTargetPivotAngle(angleLUT.getAngleFromDistance(targetDistance));

    if (readyToShoot(targetDistance, targetYaw)) {
      leds.setLEDState(LEDSubsystem.LEDState.TARGET_LOCKED);
    } else {
      leds.setLEDState(LEDSubsystem.LEDState.NO_TARGET);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Reset pivot to 60
    pivot.setTargetPivotAngle(PivotSubsystem.PivotLocations.DEG_60);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}