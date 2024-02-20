// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
    
    PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    PIDController turnPID = new PIDController(0.001, 0.00001, 0);

    boolean hasTarget; // Stores whether or not a target is detected
    PhotonPipelineResult result; // Stores all the data that Photonvision returns
    
  public VisionSubsystem() {
    var aprilTagDetector = new AprilTagDetector();
    var config = aprilTagDetector.getConfig();
    AprilTagDetector detector = new AprilTagDetector();
    detector.addFamily("tag36h11");
    

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

public boolean canSeeTargetID(int targetID) {
  var result = camera.getLatestResult();
  
  if (result.hasTargets()) {
    for (var target : result.getTargets()) {
      int fiducialId = target.getFiducialId(); 
      if (fiducialId == targetID) { 
        return true;
      }
    }
    return false;
  }
return false;
}

public double getYaw(int targetID) {
  var result = camera.getLatestResult();

  if (result.hasTargets()) {
    for (var target : result.getTargets()) {
      int fiducialId = target.getFiducialId(); 
      if (fiducialId == targetID)
      return target.getYaw(); { 
      }
    }
  }
  return 0.0;
}

public PhotonTrackedTarget getTargetWithID(int id) { // Returns the apriltag target with the specified ID (if it exists)
        List<PhotonTrackedTarget> targets = result.getTargets(); // Create a list of all currently tracked targets
        for (PhotonTrackedTarget i : targets) {
            if (i.getFiducialId() == id) { // Check the ID of each target in the list
                return i; // Found the target with the specified ID!
            }
        }
        return null; // Failed to find the target with the specified ID
    }

public PhotonTrackedTarget getBestTarget() {
        if (hasTarget) {
        return result.getBestTarget(); // Returns the best (closest) target
        }
        else {
            return null; // Otherwise, returns null if no targets are currently found
        }
    }
    public boolean getHasTarget() {
        return hasTarget; // Returns whether or not a target was found
    }

    public double getDistanceToTarget(PhotonTrackedTarget target) {
        if (!hasTarget) {
            return 0;
        }
        double april_tag_pitch = target.getPitch();
        double april_tag_area = target.getArea();

        double distance = april_tag_area;

        // Print the area and pitch of the target
        //System.out.println("Area: " + april_tag_height + "Pitch: " + april_tag_pitch);
        SmartDashboard.putNumber("t_area", april_tag_area);
        SmartDashboard.putNumber("t_pitch", april_tag_pitch);
        return distance;
    }

    public boolean InRange(double distanceThreshold, double distanceThresholdRange,
    double angleThreshold, double angleThresholdRange) {
        if (!hasTarget) {
            return false;
        }
    
        PhotonTrackedTarget bestTarget = getBestTarget();
        double distanceToTarget  = getDistanceToTarget(bestTarget);
        double angleToTarget = bestTarget.getYaw(); // Assuming yaw gives the angle
        double skewTarget = bestTarget.getSkew();

        //boolean inRange = Math.abs(distanceToTarget) <= distanceThreshold && Math.abs(angleToTarget) <= angleThreshold;
        boolean inRange = Math.abs(Math.abs(distanceToTarget) - distanceThreshold) >= distanceThresholdRange && Math.abs(Math.abs(angleToTarget) - angleThreshold) >= angleThresholdRange;
        
        SmartDashboard.putNumber("t_distance", distanceToTarget);
        SmartDashboard.putNumber("t_angle", angleToTarget);
        SmartDashboard.putNumber("t_skew", skewTarget);
        SmartDashboard.putBoolean("InRange", inRange);
    
        return inRange;
    }

@Override
  public void periodic() {
    // This method will be called once per scheduler run
    
        PhotonPipelineResult result = camera.getLatestResult(); // Query the latest result from PhotonVision
        hasTarget = result.hasTargets(); // If the camera has detected an apriltag target, the hasTarget boolean will be true
        if (hasTarget) {
            this.result = result;
        }
        InRange(0, 5, 0, 5); // Put to SmartDashboard whether or not the target is in range
    }
}
