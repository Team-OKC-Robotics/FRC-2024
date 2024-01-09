package frc.robot.subsystems;


//import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveModule frontLeft;

    private SwerveModule frontRight;

    private SwerveModule backLeft;

    private SwerveModule backRight;


    private AHRS gyro;

    
///////////////////////////////////////////////////////////////////////////////////////
// Creating my odometry object from the kinematics object and the initial wheel positions.
// Here, our starting pose is 5 meters along the long end of the field and in the
// center of the field along the short end, facing the opposing alliance wall.
    boolean Old;
    private SwerveDriveOdometry odometer;
        
////////////////////////////////////////////////////////////////////////////////////

    public SwerveSubsystem(boolean Old) {
    gyro = new AHRS(SPI.Port.kMXP);
        this.Old = Old;
        //if (Old) {  // use backup bot swerve parameters
            frontLeft = new SwerveModule(
                DriveConstants.kFrontLeftDriveMotorPort,
                DriveConstants.kFrontLeftTurningMotorPort,
                DriveConstants.kFrontLeftDriveEncoderReversed,
                DriveConstants.kFrontLeftTurningEncoderReversed,
                DriveConstants.kFrontLeftAbsoluteEncoderPort,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
                ,"FL"
                );
                frontRight = new SwerveModule(
                    DriveConstants.kFrontRightDriveMotorPort,
                    DriveConstants.kFrontRightTurningMotorPort,
                    DriveConstants.kFrontRightDriveEncoderReversed,
                    DriveConstants.kFrontRightTurningEncoderReversed,
                    DriveConstants.kFrontRightAbsoluteEncoderPort,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
                    ,"FR"
                    );
                    backLeft = new SwerveModule(
                        DriveConstants.kBackLeftDriveMotorPort,
                        DriveConstants.kBackLeftTurningMotorPort,
                        DriveConstants.kBackLeftDriveEncoderReversed,
                        DriveConstants.kBackLeftTurningEncoderReversed,
                        DriveConstants.kBackLeftAbsoluteEncoderPort,
                        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
                        ,"BL"
                        );
                        backRight = new SwerveModule(
                            DriveConstants.kBackRightDriveMotorPort,
                            DriveConstants.kBackRightTurningMotorPort,
                            DriveConstants.kBackRightDriveEncoderReversed,
                            DriveConstants.kBackRightTurningEncoderReversed,
                            DriveConstants.kBackRightAbsoluteEncoderPort,
                            DriveConstants.kBackRightDriveAbsoluteEncoderReversed
                            ,"BR"
                            );
            odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
                gyro.getRotation2d(),
                getModuleStates(), 
                new Pose2d(0.0, 0.0, new Rotation2d()));
        //} else   // use Competition bot swerve parameters

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }
    public SwerveSubsystem() {
        this(true);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
       // return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    
    /** Gets the pose in meters relative to the field coordinate system */
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    /** How far from some pose */
    public double distTravelled(Pose2d from) {
        double dist = 0.;
        return dist;
    }

    public void resetOdometry(Pose2d pose) {
        frontLeft.resetPos();
        frontRight.resetPos();
        backLeft.resetPos();
        backRight.resetPos();
        odometer.resetPosition(new Rotation2d(gyro.getAngle()),new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),  
            backLeft.getPosition(),
            backRight.getPosition()
        }    ,pose );
    }

    /** Returns position in meters
     *   of four swerve drive encoders
     */
    public double[] returnEncode() {
        double[] me = new double[] {
            -frontLeft.getDrivePosition(), 
            -frontRight.getDrivePosition(),
            -backLeft.getDrivePosition(),
            -backRight.getDrivePosition()
        };
        return me;
    }

    /** returns drive velocity in meters/sec
     *   for four drive encoders
     */
    public double[] getVelocity() {
        return new double[] {
            frontLeft.getDriveVelocity(),
            frontRight.getDriveVelocity(),
            backLeft.getDriveVelocity(),
            backRight.getDriveVelocity(),
        };
    }

    /** sets drive motor idle mode to BRAKE */
    public void setbrakemode(){
        frontLeft.setbrakemode();
        frontRight.setbrakemode();
        backLeft.setbrakemode();
        backRight.setbrakemode();
    }

    /** sets drive motor idle mode to COAST */
    public void setcoastmode(){
        frontLeft.setcoastmode();
        frontRight.setcoastmode();
        backLeft.setcoastmode();
        backRight.setcoastmode();
    }

    @Override
    public void periodic() {
        odometer.update(new Rotation2d(-gyro.getAngle()*Math.PI/180.), getModuleStates());
        //SmartDashboard.putNumber("Robot Heading", getHeading());
        //SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public SwerveModuleState[] chassis2ModuleStates(ChassisSpeeds speeds){
        return 
        DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    }

    public SwerveModulePosition [] getModuleStates() {
        return new SwerveModulePosition [] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }


    public void reportStatesToSmartDashbd(SwerveModuleState[] desiredStates) {
        frontLeft.smartDashreportState(desiredStates[0]);
        frontRight.smartDashreportState(desiredStates[1]);
        backLeft.smartDashreportState(desiredStates[2]);
        backRight.smartDashreportState(desiredStates[3]);

//       SmartDashboard.putString("Gyro: ", String.format("%.3f", gyro.getAngle()));

        //SmartDashboard.putNumber("FLabsA", frontLeft.getabsoluteEncoder());
       // SmartDashboard.putNumber("FRabsA", frontRight.getabsoluteEncoder());
       // SmartDashboard.putNumber("BLabsA", backLeft.getabsoluteEncoder());
       // SmartDashboard.putNumber("BRabsA", backRight.getabsoluteEncoder());

        // SmartDashboard.putNumber("BotX", odometer.getPoseMeters().getX());
        // SmartDashboard.putNumber("BotY", odometer.getPoseMeters().getY());

        // SmartDashboard.putNumber("FLPos", frontLeft.getDrivePosition());
        // SmartDashboard.putNumber("FRPos", frontRight.getDrivePosition());
        // SmartDashboard.putNumber("BLPos", backLeft.getDrivePosition());
        // SmartDashboard.putNumber("BRPos", backRight.getDrivePosition());

        // SmartDashboard.putNumber("FLTrn", frontLeft.getTurningPosition());
        // SmartDashboard.putNumber("FRTrn", frontRight.getTurningPosition());
        // SmartDashboard.putNumber("BLTrn", backLeft.getTurningPosition());
        // SmartDashboard.putNumber("BRTrn", backRight.getTurningPosition());
    }
    
    /**Drives in robot or field coordinate system.
     * @xhowfast and 
     * @yhowfast are the components of the vector
     * in meters/sec,
     * @turnSpeed is the rotation rate in radians/sec counterclockwise
     */
    public void driveit(double xhowfast, double yhowfast, double turnSpeed, boolean fieldoriented) {
        // Remember that xspeed is backwards on robot
        ChassisSpeeds chassisSpeeds;
        if (fieldoriented){
             chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                -xhowfast, yhowfast, turnSpeed, getRotation2d());
        } 
        else{
            chassisSpeeds = new ChassisSpeeds(-xhowfast, yhowfast,
                                         turnSpeed);
        }
        
        SwerveModuleState[] moduleStates = chassis2ModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    /**
     * Drives in only robot  coordinate system <br>
     * @xS and
     * @yS are the components of the vector
     * in meters/sec
     * <br>
     * turnSpeed is set to zero; <br>
     * fieldoriented is set to false
     */
    public void driveit(double xS, double yS) {
        driveit(xS, yS, 0., false);
    }

    public void setBrakeMode() {
        frontLeft.setbrakemode();
        frontRight.setbrakemode();
        backLeft.setbrakemode();
        backRight.setbrakemode();
    }

    public void setCoastMode() {
        frontLeft.setcoastmode();
        frontRight.setcoastmode();
        backLeft.setcoastmode();
        backRight.setcoastmode();
    }

    boolean iShouldStop = false;

    public boolean shouldistop() {
        return iShouldStop;
    }

    public void makemefalse() {
        iShouldStop = false;
    }

    public void makemestop() {
        iShouldStop = true;
    } 

    /** driveMe just stops because it is essentially unimplemented */
    public Command driveMe(double howfast){
        return runOnce(
            () -> stopModules()
        );
    }
}