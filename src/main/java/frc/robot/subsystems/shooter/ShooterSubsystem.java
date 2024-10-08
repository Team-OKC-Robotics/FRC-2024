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

  private final CANSparkMax leftShooterMotor;
  private final CANSparkMax rightShooterMotor;
  
  private final SparkPIDController RightPIDController;
  private final SparkPIDController LeftPIDController;
  private final RelativeEncoder rightEncoder;
  private final RelativeEncoder leftEncoder;
  private ShuffleboardTab tab = Shuffleboard.getTab("shooter");


  
  private GenericEntry shooterRight = tab.add("shooter right", 0.0).getEntry();
  private GenericEntry shooterLeft = tab.add("shooter leftt", 0.0).getEntry();

  public double target_Speed;

  public ShooterSubsystem() {

    leftShooterMotor = new CANSparkMax(Constants.ShooterConstants.leftShooterMotorID , CANSparkLowLevel.MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(Constants.ShooterConstants.rightShooterMotorID, CANSparkLowLevel.MotorType.kBrushless);
    

    
   
    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();
    
    
    rightShooterMotor.setInverted(false);
    leftShooterMotor.setInverted(true);

    leftShooterMotor.setClosedLoopRampRate(1.0);
    rightShooterMotor.setClosedLoopRampRate(1.0);


    leftShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    rightEncoder = rightShooterMotor.getEncoder();
    leftEncoder = leftShooterMotor.getEncoder();
    
    RightPIDController = rightShooterMotor.getPIDController();
    LeftPIDController = leftShooterMotor.getPIDController();

    RightPIDController.setOutputRange(0.0, 1.0);
    LeftPIDController.setOutputRange(0.0, 1.0);

    RightPIDController.setFeedbackDevice(rightEncoder);
    set(PIDF.PORPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE,
              PIDF.FEEDFORWARD, PIDF.INTEGRAL_ZONE);

    LeftPIDController.setFeedbackDevice(leftEncoder);
    set(PIDF.PORPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE,
              PIDF.FEEDFORWARD, PIDF.INTEGRAL_ZONE);


  }

  public static class PIDF {
    /*Feedforward constant for PID loop */
    public static final double FEEDFORWARD = 0.000199;
    /*Porportion constant for PID loop */
    public static final double PORPORTION = 0.001;
    /*Integral constant for PID loop */
    public static final double INTEGRAL = 0;
    /*Derivative constant for PID loop */
    public static final double DERIVATIVE = 0.0;
    /*Integral zone constant for PID loop */
    public static final double INTEGRAL_ZONE = 0.0;

    
  }

  public void RightshootSpeed(double power){
    //rightShooterMotor.set(power);
    //leftShooterMotor.set(power);
    //indexerMotor.set(power);
    RightPIDController.setReference(power, CANSparkMax.ControlType.kVelocity);
    
  }

  public void LeftshootSpeed(double power) {
    LeftPIDController.setReference(power, CANSparkMax.ControlType.kVelocity);
  }

  public void shootSpeed(double power) {
    RightPIDController.setReference(power, CANSparkMax.ControlType.kVelocity);
    LeftPIDController.setReference(power, CANSparkMax.ControlType.kVelocity);
  }


 

  public void stopShooter() {
    rightShooterMotor.set(0);
    leftShooterMotor.set(0);
    
    RightPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
    LeftPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
  }
  
  

 public void set(double p, double i, double d, double f, double iz) {
  LeftPIDController.setP(p);
  LeftPIDController.setI(i);
  LeftPIDController.setD(d);
  LeftPIDController.setFF(f);
  LeftPIDController.setIZone(iz);

  RightPIDController.setP(p);
  RightPIDController.setI(i);
  RightPIDController.setD(d);
  RightPIDController.setFF(f);
  RightPIDController.setIZone(iz);
 }

 public void runPID(double targetSpeed){
  target_Speed = targetSpeed;
  //PIDController.setReference(targetSpeed, CANSparkMax.ControlType.kVelocity);
 }

 public double getSpeed() {
  return rightShooterMotor.get();
  
 }

 public double getPower() {
  return rightShooterMotor.get();

 }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     shooterRight.setDouble(rightEncoder.getVelocity());
     shooterLeft.setDouble(leftEncoder.getVelocity());
// 
    // if (pidSetButton.getBoolean(false)) {
      // pidSetButton.setBoolean(false);
      // set(shooterP.getDouble(PIDF.PORPORTION), shooterI.getDouble(PIDF.INTEGRAL), PIDF.DERIVATIVE, shooterF.getDouble(PIDF.FEEDFORWARD), PIDF.INTEGRAL_ZONE);
    // }
  }

  public void RightShootIt(double speed) {
    RightPIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);

    
   // rightShooterMotor.set(speed);
   // leftShooterMotor.set(speed);
    
  }

  public void LeftShootIt(double speed) {
    LeftPIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  public void ShootIt(double speed) {
    RightPIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
    LeftPIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  public double getMinVelocity() {
    return Math.min(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }
}