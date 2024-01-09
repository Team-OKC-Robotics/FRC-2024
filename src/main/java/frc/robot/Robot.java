// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Constants.DriveConstants;

import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.cameraserver.CameraServer;


import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.net.PortForwarder;
//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.frc.AHRS;


//import com.ctre.phoenix.sensors.Pigeon2_Faults;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer = new RobotContainer();
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick secondriverJoystick = new Joystick(OIConstants.kDriverControllerPort2);
    private SwerveSubsystem swerveSubsystem;
    private AHRS gyro;
    

    private PowerDistribution PDH;
    //private AnalogInput pixyCam;
   
    private PWM lights;
    private boolean choice;  // choose which robot to control, true is competion bot
    // private Timer timer = new Timer();
    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.
        //      o build subsystems based on what is available,
        //      o perform all our button bindings,
        //      o put our autonomous chooser on the dashboard.

        choice = Preferences.containsKey("Swerve");
        //systemChooser = new AnalogInput(Constants.SYSTEMCHOOSER);
        //choice = systemChooser.getValue() == Constants.COMPBOT;
        if (choice) {
            PDH = new PowerDistribution(1, ModuleType.kRev);
        } else {
            PDH = new PowerDistribution(1, ModuleType.kCTRE);
        }
       
    
        
        gyro = m_robotContainer.getGyro();

        // DataLogManager.start();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
      
        CommandScheduler.getInstance().run();
       
        
       }
    

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
       
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        
     //   gripper = m_robotContainer.getGripperSS();  // moved to robot int
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        // CRG added stuff to drive in teleopPeriodic rather than defaultCommand {
        this.swerveSubsystem = m_robotContainer.getSwerveSS();
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.swerveSubsystem.setCoastMode();
        // }
       
    }

    double smoothedXSpeed = 0.;
    double smoothedYSpeed = 0.;
    double smoothedTurningSpeed = 0.;
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        if (driverJoystick.getRawButton(3))
            {
                System.out.println("Button Pressed");
            }

        //SmartDashboard.putNumber("BotA", pigeon.getAngle());
        SmartDashboard.putNumber("BatV", PDH.getVoltage());

        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Angle", gyro.getAngle());

        //SmartDashboard.putNumber()

        // CRG use SwerveSubsystem drive methods similar to the SwerveJoysickCmd  {
        // 1. Get real-time joystick inputs
        double xSpeed = driverJoystick.getRawAxis(OIConstants.kDriverXAxis); // Negative values go forward
        double ySpeed = -driverJoystick.getRawAxis(OIConstants.kDriverYAxis);
        double turningSpeed = -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis);

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
        //System.out.println("Deadband Applied");
        //System.out.println("X: " + String.format("%.3f", xSpeed)
        //                + " Y: " + String.format("%.3f", ySpeed)
        //                + " R: " + String.format("%.3f", turningSpeed));
        xSpeed*=1.-.8*driverJoystick.getRawAxis(OIConstants.fineControlAxis);
        ySpeed*=1.-.8*driverJoystick.getRawAxis(OIConstants.fineControlAxis);
        turningSpeed*=1.-.9*driverJoystick.getRawAxis(OIConstants.fineControlAxis);
        //    Smooth driver inputs
        smoothedXSpeed = smoothedXSpeed + (xSpeed - smoothedXSpeed) * .08;
        smoothedYSpeed = smoothedYSpeed + (ySpeed - smoothedYSpeed) * .08;
        smoothedTurningSpeed = smoothedTurningSpeed + (turningSpeed - smoothedTurningSpeed) * .08;
        //    System.out.println("Raw Joystick Values");
        //    System.out.println("X: " + String.format("%.3f", xSpeed) 
        //                    + " Y: " + String.format("%.3f", ySpeed)
        //                    + " R: " + String.format("%.3f", turningSpeed));

        if (choice?Constants.PIXY_AVAILABLE:Constants.PIXY_AVAILABLE) {
            if (driverJoystick.getRawButton(OIConstants.PixyFollowButton)){
                //int err = pixyCam.getAverageValue();
                //SmartDashboard.putNumber("PixyX",  pixyCam.getAverageValue());
                //double turnSpeedB = (err<1500)?.1:(err>1700?-0.1:0.);
                turningSpeed = Math.max(-.3,
                                Math.min(.3,
                                -2.e-4  )); 
                                //errY/22. )); 
                smoothedTurningSpeed = turningSpeed; // We're not smoothing yet
                //SmartDashboard.putNumber("turnSpeed",smoothedTurningSpeed);
            }
        }
        if (driverJoystick.getRawButton(OIConstants.BALANCE_AUGMENTER)) {
            double augment = Math.sin(Math.toRadians(gyro.getPitch()-1));
            //System.out.println(augment);
            smoothedXSpeed+=augment*.036;
        }
        xSpeed = smoothedXSpeed;
        ySpeed = smoothedYSpeed;
        turningSpeed = smoothedTurningSpeed;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        //System.out.println("Smoothing Applied");
        //System.out.println("X: " + String.format("%.3f", xSpeed)
        //                + " Y: " + String.format("%.3f", ySpeed)
        //                + " R: " + String.format("%.3f", turningSpeed));

        //System.out.println("=====================");

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if ( !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }
        //System.out.println("Chassis Speeds");
        //System.out.println("X: " + String.format("%.3f", xSpeed)
        //                + " Y: " + String.format("%.3f", ySpeed)
        //                + " R: " + String.format("%.3f", swerveSubsystem.getRotation2d()));

        //System.out.println("Encoder: " + frontleftsteerencoder.getPosition());

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = 
           DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        /* this should be done in the SwerveSubsystem
        SwerveModuleState[] moduleStates = swerveSubsystem.chassis2ModuleStates(chassisSpeeds);
        */

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
        
        // steps 4-6 should be accomplished by the swerve subsystem via a method such as
        // swerveSubsystem.driveit(xSpeed, ySpeed, turningSpeed, fieldoriented);
        //}
    
        //swerveSubsystem.reportStatesToSmartDashbd(moduleStates);

       if (Timer.getMatchTime() < 5.) {
        swerveSubsystem.setBrakeMode();
        //SmartDashboard.putNumber("Match time", Timer.getMatchTime());
       }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        
      
    }
}