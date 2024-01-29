// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
    
    PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    final CANSparkMax motor = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
    PIDController turnPID = new PIDController(0.001, 0.00001, 0);
    
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

            motor.set(turnPID.calculate(yaw));      
     
    } 
    else {
      motor.set(0);
    
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

  
@Override
  public void periodic() {
    // This method will be called once per scheduler run
    }
}
