// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private final CANSparkMax shooterMotor;
  private final CANSparkMax intakeMotor;
  
  private final SparkPIDController RightPIDController;
  private final SparkPIDController LeftPIDController;
  private final RelativeEncoder rightEncoder;
  private final RelativeEncoder leftEncoder;
  private ShuffleboardTab tab = Shuffleboard.getTab("shooter");
  private GenericEntry shooterRight = tab.add("shooter right", 0.0).getEntry();
  private GenericEntry shooterLeft = tab.add("shooter leftt", 0.0).getEntry();

  public double target_Speed;

  public ShooterSubsystem() {

    shooterMotor = new CANSparkMax(Constants.ShooterConstants.shooterMotorID , CANSparkLowLevel.MotorType.kBrushless);
    intakeMotor = new CANSparkMax(Constants.ShooterConstants.intakeMotorID, CANSparkLowLevel.MotorType.kBrushless);

    rightEncoder = intakeMotor.getEncoder();
    leftEncoder = shooterMotor.getEncoder();
    
    RightPIDController = intakeMotor.getPIDController();
    LeftPIDController = shooterMotor.getPIDController();

  }

  public void setShooterSpeed(double speed) {
    shooterMotor.set(-speed);

  }

  public void stopShooter() {
    shooterMotor.set(0);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }
  public void stopIntake() {
    intakeMotor.set(0);
  }

}



 