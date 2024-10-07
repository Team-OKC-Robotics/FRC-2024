// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.vision;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.LerpedLUT;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class AutoAim extends Command {
  /** Creates a new AutoAim. */
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final PivotSubsystem pivot;
  private int targetAprilTag = 4;

  VisionSubsystem visionSubsystem = new VisionSubsystem();
  LerpedLUT angleLUT = new LerpedLUT();

  private ShuffleboardTab tab = Shuffleboard.getTab("shooter");

  private GenericEntry distanceEntry = tab.add("Distance To Tag", 0.0).getEntry();
  private GenericEntry idealAngleEntry = tab.add("PivotAngle", 0.0).getEntry();
  private GenericEntry idealYaw = tab.add("Ideal Yaw", 0.0).getEntry();
  private GenericEntry targetSpeakerID = tab.add("Target Speaker ID", 0).getEntry();

  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;

  private double lastDistance = 0.0;
  private double lastYaw = 0.0;
  private boolean isRunning = false;

  public AutoAim(SwerveSubsystem swerve, VisionSubsystem vision, PivotSubsystem pivot, DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(swerve, vision, pivot);

    this.swerve = swerve;
    this.vision = vision;
    this.pivot = pivot;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      this.targetAprilTag = 7;
    }

    targetSpeakerID.setInteger(this.targetAprilTag);

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

  public boolean readyToShoot() {
    return lastDistance < 5.7 && lastYaw < 3 && pivot.isPivotAtSetpoint();
  }

  public boolean isRunning() {
    return this.isRunning;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset last values to make LEDs more accurate
    lastDistance = 20;
    lastYaw = 20;
    isRunning = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    PhotonTrackedTarget target = vision.getTargetWithID(targetAprilTag);
    // vision.getTargetWithID(4);

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(),
        swerve.getHeading().getSin(), swerve.getHeading().getCos());

    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
        Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
        swerve.getSwerveDriveConfiguration());

    if (target == null) {
      swerve.drive(translation, 0, true);
      idealYaw.setDouble(2718.0);
    } else {
      double yaw = target.getYaw();
      idealYaw.setDouble(yaw);
      lastYaw = yaw;
      if (Math.abs(yaw) > 2) {
        swerve.drive(translation, -0.10 * yaw, true);
      } else {
        swerve.drive(translation, 0, true);
      }
    }

    double distance = Units.metersToFeet(visionSubsystem.distanceToTarget(target, tagHeight, cameraHeight, cameraAngle));
    distance = distance - 3.9; // Camera + robot offset

    this.lastDistance = distance;

    double idealAngle = angleLUT.getAngleFromDistance(distance);

    distanceEntry.setDouble(distance);
    idealAngleEntry.setDouble(idealAngle);

    pivot.SetTargetPivotAngle(idealAngle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Reset last values to make LEDs more accurate
    lastDistance = 20;
    lastYaw = 20;
    isRunning = false;

    // Reset pivot to "60"
    pivot.SetTargetPivotAngle(57);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // if (vision.getTargetWithID(targetAprilTag) == null) {
    // return false;
    // } else {
    // return Math.abs(vision.getTargetWithID(targetAprilTag).getYaw()) < 1;
    // }
  }
}
