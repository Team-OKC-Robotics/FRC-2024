// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final boolean COMPETITION_MODE = false;

  public static final double ROBOT_MASS = Units.lbsToKilograms(120);
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(15.1);

  public static class OperatorConstants
  {
    public static final double DEADBAND = 0.25;
    public static final double TURN_CONSTANT = 0.75;
  }

  public static final class ShooterConstants {
    public static final int leftShooterMotorID = 14;
    public static final int rightShooterMotorID = 13;
    public static final int indexerMotorID = 10;
    
  }

  public static final class IntakeConstants
  {
    public static final int intakeMotorID = 9;
    public static final int limitSwitchBeamBrakeChannel = 1;
    public static final int intakeLimitSwitchChannel = 8;
  }
  
  public static final class ClimberConstants 
  {
    public static final int rightClimberMotorID = 15;
    public static final int leftClimberMotorID = 16;
  }
  public static final class PivotConstants {
    public static final int pivotMotorID = 11;
  }

  public static final class LEDS {
    public static final int PWMPort = 5;
    public static final int Length = 20;
  }
}