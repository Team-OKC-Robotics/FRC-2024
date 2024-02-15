package frc.robot.subsystems.shooter;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private final CANSparkMax leftShooterMotor;
  private final CANSparkMax rightShooterMotor;
  private final CANSparkMax indexerMotor;
  private final SparkPIDController PIDController;
  private final RelativeEncoder rightEncoder;
  private final RelativeEncoder leftEncoder;
  
  public double target_Speed;

  public ShooterSubsystem() {

    leftShooterMotor = new CANSparkMax(Constants.ShooterConstants.leftShooterMotorID , CANSparkLowLevel.MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(Constants.ShooterConstants.rightShooterMotorID, CANSparkLowLevel.MotorType.kBrushless);
    indexerMotor = new CANSparkMax(Constants.ShooterConstants.indexerMotorID, CANSparkLowLevel.MotorType.kBrushless);

    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();
    indexerMotor.restoreFactoryDefaults();
    leftShooterMotor.follow(rightShooterMotor);
    rightShooterMotor.setInverted(true);
    leftShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightEncoder = rightShooterMotor.getEncoder();
    leftEncoder = leftShooterMotor.getEncoder();
    PIDController = rightShooterMotor.getPIDController();
    PIDController.setFeedbackDevice(rightEncoder);
    set(PIDF.PORPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE,
              PIDF.FEEDFORWARD, PIDF.INTEGRAL_ZONE);


  }

  public static class PIDF {
    /*Feedforward constant for PID loop */
    public static final double FEEDFORWARD = 0.01;
    /*Porportion constant for PID loop */
    public static final double PORPORTION = 0.05;
    /*Integral constant for PID loop */
    public static final double INTEGRAL = 0.0;
    /*Derivative constant for PID loop */
    public static final double DERIVATIVE = 0.0;
    /*Integral zone constant for PID loop */
    public static final double INTEGRAL_ZONE = 0.0;
  }

  public void shootSpeed(double power){
    rightShooterMotor.set(power);
    indexerMotor.set(power);
  }

  public void stop() {
    rightShooterMotor.set(0);
    indexerMotor.set(0);
  }
  
  

 public void set(double p, double i, double d, double f, double iz) {
  PIDController.setP(p);
  PIDController.setI(i);
  PIDController.setD(d);
  PIDController.setFF(f);
  PIDController.setIZone(iz);
 }

 public void runPID(double targetSpeed){
  target_Speed = targetSpeed;
  PIDController.setReference(targetSpeed, CANSparkMax.ControlType.kVelocity);
 }

 public double getSpeed() {
  return rightShooterMotor.get();
 }

 public double getPower() {
  return rightShooterMotor.get();

 }

 public Command shootIt(double Speed) {
  return run(() -> runPID(Speed));
 }

 public enum ShooterState{
  HIGHPOWER(100),
  MIDPOWER(50),
  LOWPOWER(25),
  OFF(0);

  public double speed;

  private ShooterState(double speed) {
    this.speed = speed;
  }
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ShootIt(double speed) {
    rightShooterMotor.set(speed);
    indexerMotor.set(speed);
  }
}