// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

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

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.3;
    public static final double LEFT_Y_DEADBAND = 0.3;
    public static final double RIGHT_X_DEADBAND = 0.3;
    public static final double TURN_CONSTANT = 0.75;
  }

  public static final class ShooterConstants {
    public static final int leftShooterMotorID = 14;
    public static final int rightShooterMotorID = 13;
    public static final int indexerMotorID = 10;
  }

  public static final class IntakeConstants
  {
    public static final int intakemotorID = 9;
    public static final int limitSwitchBeanBrakeChannel = 1;
  }

  public static final class PivotConstants {
    public static final int pivotmotorID = 11;
  }

  public static class OI {
    //second driver buttons
    public static final int kSecondriverButton2 = 2;
    public static final int kSecondriverButton4 = 4;
    public static final int kSecondriverButton1 = 1;
    public static final int kSecondriverButton3 = 3;
    public static final int kSecondriverButton5 = 5;
    public static final int kSecondriverButton6 = 6;



    // first driver buttons
    public static final int kdriverControllerButton2 = 2;
    public static final int kdriverControllerButton5 = 5;
    

  }
}