package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final double leftMotorShootSetpoint = 5500;
  private final double rightMotorShootSetpoint = 5000;
  private final double velocityTolerance = 100;

  private final SparkMax leftShooterMotor;
  private final SparkMax rightShooterMotor;
  private final SparkClosedLoopController RightPIDController;
  private final SparkClosedLoopController LeftPIDController;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  public final Trigger atSpeed = new Trigger(this::atSpeed);

  public ShooterSubsystem() {

    leftShooterMotor = new SparkMax(Constants.ShooterConstants.leftShooterMotorID, MotorType.kBrushless);
    rightShooterMotor = new SparkMax(Constants.ShooterConstants.rightShooterMotorID, MotorType.kBrushless);

    leftEncoder = leftShooterMotor.getEncoder();
    rightEncoder = rightShooterMotor.getEncoder();

    SparkMaxConfig leftconfig = new SparkMaxConfig();
    SparkMaxConfig rightconfig = new SparkMaxConfig();

    leftconfig.inverted(true).idleMode(IdleMode.kCoast).closedLoopRampRate(1.0).openLoopRampRate(1.0);
    leftconfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(0.0, 1.0)
        .pidf(PIDF.PORPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE, PIDF.FEEDFORWARD);

    rightconfig.apply(leftconfig);
    rightconfig.inverted(false);

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

  public Command runContinously() {
    return run(() -> setVelocityReference(RPM.of(5500), RPM.of(5000)));
  }

  public Command spinUp() {
    return run(() -> setVelocityReference(RPM.of(5500), RPM.of(5000))).until(atSpeed);
  }

  public Command stop() {
    return run(() -> setVelocityReference(RPM.of(0), RPM.of(0)));
  }

  public Command setVoltage(double voltage) {
    return run(() -> setVoltageReference(Volts.of(voltage)));
  }

  private boolean atSpeed() {
    return MathUtil.isNear(leftEncoder.getVelocity(), leftMotorShootSetpoint, velocityTolerance)
        && MathUtil.isNear(rightEncoder.getVelocity(), rightMotorShootSetpoint, velocityTolerance);
  }

  private void setVelocityReference(AngularVelocity left_velocity, AngularVelocity right_velocity) {
    RightPIDController.setReference(left_velocity.in(RPM), SparkMax.ControlType.kVelocity);
    LeftPIDController.setReference(right_velocity.in(RPM), SparkMax.ControlType.kVelocity);
  }

  private void setVoltageReference(Voltage voltage) {
    RightPIDController.setReference(voltage.in(Volts), SparkMax.ControlType.kVoltage);
    LeftPIDController.setReference(voltage.in(Volts), SparkMax.ControlType.kVoltage);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Right Encoder Velocity", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Left Encoder Velocity", leftEncoder.getVelocity());
  }
}