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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public ShooterSubsystem() {

    leftShooterMotor = new SparkMax(Constants.ShooterConstants.leftShooterMotorID, MotorType.kBrushless);
    rightShooterMotor = new SparkMax(Constants.ShooterConstants.rightShooterMotorID, MotorType.kBrushless);

    leftEncoder = leftShooterMotor.getEncoder();
    rightEncoder = rightShooterMotor.getEncoder();

    SparkMaxConfig leftconfig = new SparkMaxConfig();
    SparkMaxConfig rightconfig = new SparkMaxConfig();

    leftconfig.inverted(true).idleMode(IdleMode.kCoast).closedLoopRampRate(1.0).openLoopRampRate(1.0);
    rightconfig.inverted(false).idleMode(IdleMode.kCoast).closedLoopRampRate(1.0).openLoopRampRate(1.0);

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
    /* Feedforward constant for PID loop */
    public static final double FEEDFORWARD = 0.000199;
    /* Porportion constant for PID loop */
    public static final double PORPORTION = 0.001;
    /* Integral constant for PID loop */
    public static final double INTEGRAL = 0;
    /* Derivative constant for PID loop */
    public static final double DERIVATIVE = 0.0;
  }

  public void setRightMotorRPM(double rpm) {
    RightPIDController.setReference(rpm, SparkMax.ControlType.kVelocity);

  }

  public void setLeftMotorRPM(double rpm) {
    LeftPIDController.setReference(rpm, SparkMax.ControlType.kVelocity);
  }

  public void setRPM(double rpm) {
    setLeftMotorRPM(rpm);
    setRightMotorRPM(rpm);
  }

  public void stopShooter() {
    RightPIDController.setReference(0, SparkMax.ControlType.kVelocity);
    LeftPIDController.setReference(0, SparkMax.ControlType.kVelocity);
  }

  public double getSpeed() {
    return rightShooterMotor.get();

  }

  public double getPower() {
    return rightShooterMotor.get();

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Right Encoder Velocity", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Left Encoder Velocity", leftEncoder.getVelocity());
  }

  public void RightShootIt(double speed) {
    RightPIDController.setReference(speed, SparkMax.ControlType.kVelocity);
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