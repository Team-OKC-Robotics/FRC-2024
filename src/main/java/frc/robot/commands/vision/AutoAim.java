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

  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;

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

      angleLUT.addEntry(-100, 60);
      angleLUT.addEntry(0, 58); // distance in feet, angle in degrees
      angleLUT.addEntry(2.17, 43);
      angleLUT.addEntry(3.37, 38);
      angleLUT.addEntry(4.33, 32.7);
      angleLUT.addEntry(5.33, 29);
      angleLUT.addEntry(6.33, 27.5);
    }
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
    } else {
      double yaw = vision.getYaw(targetAprilTag);
      swerve.drive(translation, -0.1 * yaw, true);
    }

    double distance = Units.metersToFeet(
        visionSubsystem.distanceToTarget(tagHeight, cameraHeight, cameraAngle, distanceThreshold, angleThreshold));
    distance = distance - 3.9; // Camera + robot offset
    double idealAngle = angleLUT.getAngleFromDistance(distance);

    distanceEntry.setDouble(distance);
    idealAngleEntry.setDouble(idealAngle);

    pivot.SetTargetPivotAngle(idealAngle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
