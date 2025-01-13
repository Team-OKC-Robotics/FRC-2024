package frc.robot.subsystems.shooter;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private final SparkMax leftShooterMotor;
  private final SparkMax rightShooterMotor;
  private final SparkClosedLoopController RightPIDController;
  private final SparkClosedLoopController LeftPIDController;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private ShuffleboardTab tab = Shuffleboard.getTab("shooter");

  private GenericEntry shooterRight = tab.add("shooter right", 0.0).getEntry();
  private GenericEntry shooterLeft = tab.add("shooter leftt", 0.0).getEntry();

  public double target_Speed;

  public ShooterSubsystem() {

    leftShooterMotor = new SparkMax(Constants.ShooterConstants.leftShooterMotorID, MotorType.kBrushless);
    rightShooterMotor = new SparkMax(Constants.ShooterConstants.rightShooterMotorID, MotorType.kBrushless);

    leftEncoder = leftShooterMotor.getEncoder();
    rightEncoder = rightShooterMotor.getEncoder();
    
    SparkMaxConfig leftconfig = new SparkMaxConfig();
    SparkMaxConfig rightconfig = new SparkMaxConfig();

    leftconfig.inverted(true).idleMode(IdleMode.kCoast).closedLoopRampRate(1.0);
    rightconfig.inverted(false).idleMode(IdleMode.kCoast).closedLoopRampRate(1.0);

    leftconfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(0.0, 1.0)
      .pidf(PIDF.PORPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE, PIDF.FEEDFORWARD);
    
    rightconfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(0.0, 1.0)
      .pidf(PIDF.PORPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE, PIDF.FEEDFORWARD);

    leftShooterMotor.configure(leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightShooterMotor.configure(rightconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    RightPIDController = rightShooterMotor.getClosedLoopController();
    LeftPIDController = leftShooterMotor.getClosedLoopController();
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
  }

  public void RightshootSpeed(double power){
    //rightShooterMotor.set(power);
    //leftShooterMotor.set(power);
    //indexerMotor.set(power);
    RightPIDController.setReference(power, SparkMax.ControlType.kVelocity);
    
  }

  public void LeftshootSpeed(double power) {
    LeftPIDController.setReference(power, SparkMax.ControlType.kVelocity);
  }

  public void shootSpeed(double power) {
    RightPIDController.setReference(power, SparkMax.ControlType.kVelocity);
    LeftPIDController.setReference(power, SparkMax.ControlType.kVelocity);
  }


 

  public void stopShooter() {
    rightShooterMotor.set(0);
    leftShooterMotor.set(0);
    
    RightPIDController.setReference(0, SparkMax.ControlType.kVelocity);
    LeftPIDController.setReference(0, SparkMax.ControlType.kVelocity);
  }


 public void runPID(double targetSpeed){
  target_Speed = targetSpeed;
  //PIDController.setReference(targetSpeed, SparkMax.ControlType.kVelocity);
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
    RightPIDController.setReference(speed, SparkMax.ControlType.kVelocity);

    
   // rightShooterMotor.set(speed);
   // leftShooterMotor.set(speed);
    
  }

  public void LeftShootIt(double speed) {
    LeftPIDController.setReference(speed, SparkMax.ControlType.kVelocity);
  }

  public void ShootIt(double speed) {
    RightPIDController.setReference(speed, SparkMax.ControlType.kVelocity);
    LeftPIDController.setReference(speed, SparkMax.ControlType.kVelocity);
  }

  public double getMinVelocity() {
    return Math.min(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }
}