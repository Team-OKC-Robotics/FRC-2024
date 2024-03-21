// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, robotToCam);
    PIDController turnPID = new PIDController(0.001, 0.00001, 0);

    boolean hasTarget; // Stores whether or not a target is detected
    PhotonPipelineResult result; // Stores all the data that Photonvision returns

    
    
  public VisionSubsystem() {
    var aprilTagDetector = new AprilTagDetector();
    var config = aprilTagDetector.getConfig();
    AprilTagDetector detector = new AprilTagDetector();
    detector.addFamily("tag36h11");
    
    List<PhotonTrackedTarget> targets = getTargetsByIds(new int[] {4, 5, 6, 7});

  }

  public void updateVisionData() {
    
    if (camera != null) {
    var result = camera.getLatestResult();
  
    if (result.hasTargets()) {}
            // Get the fiducial ID and yaw
            
            double fiducialId = result.getBestTarget().getFiducialId();
            double yaw = result.getBestTarget().getYaw();

            // Print the information to the terminal
            System.out.println("Fiducial ID: " + fiducialId);
            System.out.println("Yaw: " + yaw);

            // You can also use SmartDashboard to display the information on the Shuffleboard
            SmartDashboard.putNumber("Fiducial ID", fiducialId);
            SmartDashboard.putNumber("Yaw", yaw);
            
 }

}

public boolean canSeeTargetID(int[] targets) {
    var result = camera.getLatestResult();
  
    if (result.hasTargets()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
            int fiducialId = target.getFiducialId(); 
            for (int id : targets) {
                if (fiducialId == id) { 
                    return true;
                }
            }
        }
    }
    return false;
}

public double getYaw(int fiducialIdToFind) {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == fiducialIdToFind) {
                return target.getYaw();
            }
        }
    }
    return 0.0;
}

public List<PhotonTrackedTarget> getTargetsByIds(int[] ids) {
        if (result == null) {
            return new ArrayList<>(); // Return an empty list if no result is available
        }

        List<PhotonTrackedTarget> targets = result.getTargets();
        List<PhotonTrackedTarget> filteredTargets = new ArrayList<>();

        for (PhotonTrackedTarget target : targets) {
            if (Arrays.stream(ids).anyMatch(id -> id == target.getFiducialId())) {
                filteredTargets.add(target);
            }
        }

        return filteredTargets;
    }

    public PhotonTrackedTarget getBestTarget() {
        if (hasTarget) {
            return result.getBestTarget(); // Returns the best (closest) target
        } else {
            return null; // Otherwise, returns null if no targets are currently found
        }
    }
    public boolean getHasTarget() {
        return hasTarget; // Returns whether or not a target was found
    }

    public double distanceToTarget(double tagHeight, double cameraHeight, double cameraAngle, double distanceThreshold,
    double angleThreshold) {
    // convert angle to radians
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(50); // placeholder
    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(57.13);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0); // placeholder

    double angleRadians = Math.toRadians(cameraAngle);
    double distance = (tagHeight - cameraHeight) / Math.tan(angleRadians);

    if (result.hasTargets()) {
                // First calculate range
                double range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS, 
                                Units.degreesToRadians(result.getBestTarget().getPitch())); 
    
    ShuffleboardTab tab = Shuffleboard.getTab("Vision"); 
    tab.add("Range", range);
    }
    return distance;
  }

    

@Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    PhotonPipelineResult result = camera.getLatestResult(); // Query the latest result from PhotonVision
    hasTarget = result.hasTargets(); // If the camera has detected an apriltag target, the hasTarget boolean will be true
    if (hasTarget) {
      this.result = result;
    }
  }
}