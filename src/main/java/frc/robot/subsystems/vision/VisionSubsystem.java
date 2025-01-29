// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(25);
  private final double TARGET_HEIGHT_METERS = Units.inchesToMeters(57.13); // Speaker AprilTag Height
  private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(30);

  // PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");

  PhotonPipelineResult latestResult;

  public VisionSubsystem() {
  };

  public PhotonTrackedTarget getTargetWithID(int fiducialId) {
    if (latestResult == null) {
      return null;
    }

    for (PhotonTrackedTarget target : latestResult.getTargets()) {
      if (target.getFiducialId() == fiducialId) {
        return target;
      }
    }

    return null;
  }

  public double distanceToTarget(PhotonTrackedTarget target, double tagHeight, double cameraHeight,
      double cameraAngle) {

    if (target == null) {
      return 1000; // Set very high so LUT commands the pivot down
    }

    return PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT_METERS,
        TARGET_HEIGHT_METERS,
        CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(target.getPitch()));
  }

  @Override
  public void periodic() {
    // if (!camera.isConnected()) {
    //   return;
    // }
    // List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    // if (results.size() > 0) {
    //   latestResult = results.get(0);
    // }
  }

}
