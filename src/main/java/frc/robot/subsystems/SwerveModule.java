package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.CANCoderConfiguration;
//import com.ctre.phoenix.sensors.CANCoderFaults;
//import com.ctre.phoenix.sensors.CANCoderStickyFaults;
//import com.ctre.phoenix.sensors.MagnetFieldStrength;


import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;
  
    private final AnalogEncoder absoluteEncoder;
    // private final PIDController turningPidController;
    private final SparkPIDController turningPidController;
    private final SparkPIDController drivPidController;
    private final double absoluteoffset;

   
    private final boolean absoluteEncoderReversed;

    private String reportName="";
    private boolean comp=false; // true for competition bot

   // public SwerveModule(int driveMotorId, int turningMotorId, 
   //         boolean driveMotorReversed, boolean turningMotorReversed,
   //         int absoluteEncoderId, boolean absoluteEncoderReversed,
   //         String name) {
   //     this(driveMotorId, turningMotorId, 
   //         driveMotorReversed, turningMotorReversed,
   //         absoluteEncoderId, absoluteEncoderReversed);
   //     this.reportName = name;
   //     this.comp=false;
   // }


    public SwerveModule(int driveMotorId, int turningMotorId, 
            boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, boolean absoluteEncoderReversed, double absoluteEncoderoffset,
            String name) {

        this.reportName = name;
            
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        absoluteEncoder = new AnalogEncoder(absoluteEncoderId);
        absoluteoffset = absoluteEncoderoffset;
        

        driveEncoder.setPositionConversionFactor(
            
            ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(
           
            ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(
            
            ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(
            
            ModuleConstants.kTurningEncoderRPM2RadPerSec);

        // turningPidController = new PIDController(0.0005, 0, 0.00005);
        turningPidController = turningMotor.getPIDController();
        turningPidController.setP( 0.5);
        turningPidController.setI( 0.);
        turningPidController.setFF( 0.);
        turningPidController.setD( 0.0);
        turningPidController.setOutputRange(-1., 1.);

        drivPidController = driveMotor.getPIDController();
        drivPidController.setP(0.5);
        drivPidController.setI(0.0);
        drivPidController.setFF(0.);
        drivPidController.setD(0.0);
        drivPidController.setOutputRange(-1, absoluteEncoderoffset);


        // absoluteEncoder.setPositionOffset(absoluteEncoderoffset);

        resetEncoders();
    }

    public SwerveModulePosition getPosition() {   
      return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    } 

    /** Returns drive position in meters
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition(); //TODO ABBEY IS BAD
    }
    public double getTurningPosition() {
        // return absoluteEncoder.getAbsolutePosition()*360  + absoluteoffset; //converts rotations to degrees
        return turningEncoder.getPosition();
    
    }

    /** returns drive velocity in meters/sec */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }
    
    public double getDriveTemperature() {
        return driveMotor.getMotorTemperature(); 
    }
    public double getTurningTemperature() {
        return turningMotor.getMotorTemperature(); 
    }
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(absoluteEncoder.getAbsolutePosition()* 2 *Math.PI + absoluteoffset); //FIXME
       // turningEncoder.setPosition(0);
    }

    public void resetPos() {
        
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /** sets drive motor idle mode to COAST */
    public void setcoastmode() {
        driveMotor.setIdleMode(IdleMode.kCoast);
    }

    /** sets drive motor idle mode to BRAKE */
    public void setbrakemode() {
        driveMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / 
            (comp?DriveConstants.kPhysicalMaxSpeedMetersPerSecond:
                  DriveConstants.kPhysicalMaxSpeedMetersPerSecond));
                //   turningPidController.setSetpoint(state.angle.getDegrees());
                //   turningMotor.set(turningPidController.calculate(getTurningPosition()));
        turningPidController.setReference(state.angle.getRadians(),ControlType.kPosition);
    //  SmartDashboard.putString("Swerve[" + absoluteEncoder.getAbsolutePosition() + "] state", state.toString());
     //    SmartDashboard.putString("Swerve[" + this.reportName + "] state", state.toString());
    //  SmartDashboard.putNumber("Setpoint", turningPidController.getSetpoint());
      
    }

    public void smartDashreportState(SwerveModuleState state) {
         SmartDashboard.putNumber(reportName+ " ABS Encoder" ,absoluteEncoder.getAbsolutePosition()*2*Math.PI);
        // SmartDashboard.putNumber(reportName+ " Encoder" ,turningEncoder.getPosition());
    }


    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
