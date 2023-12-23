package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.CANCoderConfiguration;
//import com.ctre.phoenix.sensors.CANCoderFaults;
//import com.ctre.phoenix.sensors.CANCoderStickyFaults;
//import com.ctre.phoenix.sensors.MagnetFieldStrength;


import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;
    private final SparkMaxPIDController turningPidController;

   
    private final boolean absoluteEncoderReversed;

    private String reportName="";
    private boolean comp=false; // true for competition bot

    public SwerveModule(int driveMotorId, int turningMotorId, 
            boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, boolean absoluteEncoderReversed,
            String name) {
        this(driveMotorId, turningMotorId, 
            driveMotorReversed, turningMotorReversed,
            absoluteEncoderId, absoluteEncoderReversed);
        this.reportName = name;
        this.comp=false;
    }
    public SwerveModule(int driveMotorId, int turningMotorId, 
        boolean driveMotorReversed, boolean turningMotorReversed,
        int absoluteEncoderId, boolean absoluteEncoderReversed,
        boolean comp) {
        this(driveMotorId, turningMotorId, 
            driveMotorReversed, turningMotorReversed,
            absoluteEncoderId, absoluteEncoderReversed);
        this.comp = comp;
    }
    public SwerveModule(int driveMotorId, int turningMotorId, 
        boolean driveMotorReversed, boolean turningMotorReversed,
        int absoluteEncoderId, boolean absoluteEncoderReversed,
        String name,
        boolean comp) {
        this(driveMotorId, turningMotorId, 
            driveMotorReversed, turningMotorReversed,
            absoluteEncoderId, absoluteEncoderReversed);
        this.comp = comp;
        this.reportName=name;
    }

    public SwerveModule(int driveMotorId, int turningMotorId, 
            boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, boolean absoluteEncoderReversed) {

        this.absoluteEncoderReversed = absoluteEncoderReversed;

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(
            
            ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(
           
            ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(
            
            ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(
            
            ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = turningMotor.getPIDController();
        turningPidController.setP( 2.0);
        turningPidController.setI( 0.);
        turningPidController.setFF( 0.);
        turningPidController.setD( 0.);
        turningPidController.setOutputRange(-1., 1.);

        resetEncoders();
    }

    public SwerveModulePosition getPosition() {   
      return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    } 

    /** Returns drive position in meters
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }
    public double getTurningPosition() {
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
        turningEncoder.setPosition(0);
    }

    public void resetPos() {
        driveEncoder.setPosition(0.);
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
            (comp?DriveConstants.kPhysicalMaxSpeedMetersPerSecond_Comp:
                  DriveConstants.kPhysicalMaxSpeedMetersPerSecond));
        turningPidController.setReference(state.angle.getRadians(),ControlType.kPosition);
//        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
    }

    public void smartDashreportState(SwerveModuleState state) {
        SmartDashboard.putNumber(reportName+ " Encoder" ,turningEncoder.getPosition());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
