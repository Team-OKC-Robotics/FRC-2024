package frc.robot.swervelib.parser.json;

import com.revrobotics.SparkRelativeEncoder.Type;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.swervelib.encoders.AnalogAbsoluteEncoderSwerve;



import frc.robot.swervelib.encoders.SparkMaxAnalogEncoderSwerve;
import frc.robot.swervelib.encoders.SparkMaxEncoderSwerve;
import frc.robot.swervelib.encoders.SwerveAbsoluteEncoder;


import frc.robot.swervelib.imu.NavXSwerve;

import frc.robot.swervelib.imu.SwerveIMU;


import frc.robot.swervelib.motors.SparkMaxSwerve;
import frc.robot.swervelib.motors.SwerveMotor;


/**
 * Device JSON parsed class. Used to access the JSON data.
 */
public class DeviceJson
{

  /**
   * The device type, e.g. pigeon/pigeon2/sparkmax/talonfx/navx
   */
  public String type;
  /**
   * The CAN ID or pin ID of the device.
   */
  public int    id;
  /**
   * The CAN bus name which the device resides on if using CAN.
   */
  public String canbus = "";

  /**
   * Create a {@link SwerveAbsoluteEncoder} from the current configuration.
   *
   * @param motor {@link SwerveMotor} of which attached encoders will be created from, only used when the type is
   *              "attached" or "canandencoder".
   * @return {@link SwerveAbsoluteEncoder} given.
   */
  public SwerveAbsoluteEncoder createEncoder(SwerveMotor motor)
  {
    if (id > 40)
    {
      DriverStation.reportWarning("CAN IDs greater than 40 can cause undefined behaviour, please use a CAN ID below 40!",
                                  false);
    }
    switch (type)
    {
      case "none":
        return null;
      case "integrated":
      case "attached":
        return new SparkMaxEncoderSwerve(motor, 1);
      case "sparkmax_analog":
        return new SparkMaxAnalogEncoderSwerve(motor);
      case "canandcoder":
        return new SparkMaxEncoderSwerve(motor, 360);
     
      case "ma3":
      case "ctre_mag":
      case "rev_hex":
      case "throughbore":
      case "am_mag":
      
      case "thrifty":
      case "analog":
        return new AnalogAbsoluteEncoderSwerve(id);
      
      default:
        throw new RuntimeException(type + " is not a recognized absolute encoder type.");
    }
  }

  /**
   * Create a {@link SwerveIMU} from the given configuration.
   *
   * @return {@link SwerveIMU} given.
   */
  public SwerveIMU createIMU()
  {
    if (id > 40)
    {
      DriverStation.reportWarning("CAN IDs greater than 40 can cause undefined behaviour, please use a CAN ID below 40!",
                                  false);
    }
    switch (type)
    {
      
      case "navx":
      case "navx_spi":
        return new NavXSwerve(SPI.Port.kMXP);
      case "navx_i2c":
        DriverStation.reportWarning(
            "WARNING: There exists an I2C lockup issue on the roboRIO that could occur, more information here: " +
            "\nhttps://docs.wpilib.org/en/stable/docs/yearly-overview/known-issues" +
            ".html#onboard-i2c-causing-system-lockups",
            false);
        return new NavXSwerve(I2C.Port.kMXP);
      case "navx_usb":
        return new NavXSwerve(Port.kUSB);
      case "navx_mxp":
        return new NavXSwerve(Port.kMXP);
      
      default:
        throw new RuntimeException(type + " is not a recognized imu/gyroscope type.");
    }
  }

  /**
   * Create a {@link SwerveMotor} from the given configuration.
   *
   * @param isDriveMotor If the motor being generated is a drive motor.
   * @return {@link SwerveMotor} given.
   */
  public SwerveMotor createMotor(boolean isDriveMotor)
  {
    if (id > 40)
    {
      DriverStation.reportWarning("CAN IDs greater than 40 can cause undefined behaviour, please use a CAN ID below 40!",
                                  false);
    }
    switch (type)
    {
      case "sparkmax_brushed":
        switch (canbus)
        {
          default:
            if (isDriveMotor)
            {
              throw new RuntimeException("Spark MAX " + id + " MUST have a encoder attached to the motor controller.");
            }
            // We are creating a motor for an angle motor which will use the absolute encoder attached to the data port.
           
        }
      case "neo":
      case "sparkmax":
        return new SparkMaxSwerve(id, isDriveMotor);
      
      default:
        throw new RuntimeException(type + " is not a recognized motor type.");
    }
  }
}
