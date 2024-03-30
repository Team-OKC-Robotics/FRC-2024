// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.intake.BackwardIntake;
import frc.robot.commands.intake.SetIntakeCommand;
import frc.robot.commands.intake.SetIntakeCommandAuto;
import frc.robot.commands.pivot.PivotOtherway;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.commands.pivot.SetPivotCommand;
import frc.robot.commands.shooter.ShootWait;
import frc.robot.commands.shooter.ShootWaitAuto;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.shooter.SpinUpAuto;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.vision.AutoAim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.*;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.commands.vision.AutoAimInAuto;
import frc.robot.commands.AmpDevice.*;
import frc.robot.utils.POVButton;
import frc.robot.commands.climber.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/swerve"));

  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final PivotSubsystem m_pivot = new PivotSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final LEDSubsystem m_leds = new LEDSubsystem();

  //controllers
  XboxController driverXbox = new XboxController(0); 

  XboxController secondriverXbox = new XboxController(1);
 
  private final Joystick driverController = new Joystick(0);
  private final Joystick secondriver = new Joystick(1);
 
  //driver buttons

  private final JoystickButton driverControllerButtonB = new JoystickButton(driverController, Constants.OI.kdriverControllerButton2);
  private final JoystickButton driverControllerleftbumper = new JoystickButton(driverController, Constants.OI.kdriverControllerButton5);
  private final JoystickButton driverControllerrightbumper = new JoystickButton(driverController, Constants.OI.kdriverControllerButton6);
  
  //second driver buttons
  private final JoystickButton secondriverButtonB = new JoystickButton(secondriver, Constants.OI.kSecondriverButton2);
  private final JoystickButton secondriverButtonY = new JoystickButton(secondriver, Constants.OI.kSecondriverButton4);
  private final JoystickButton secondriverButtonA = new JoystickButton(secondriver, Constants.OI.kSecondriverButton1);
  private final JoystickButton secondriverButtonX = new JoystickButton(secondriver, Constants.OI.kSecondriverButton3);
  private final JoystickButton secondriverleftbumper = new JoystickButton(secondriver, Constants.OI.kSecondriverButton5);
  private final JoystickButton secondriverrightbumper = new JoystickButton(secondriver, Constants.OI.kSecondriverButton6);
  private final JoystickButton secondriverButton7 = new JoystickButton(secondriver, Constants.OI.kSecondriverButton7);
  private final JoystickButton secondriverButton8 = new JoystickButton(secondriver, Constants.OI.kSecondriverButton8);
 
  
  // shooter commands
  private final ShooterCommand runShooter = new ShooterCommand(m_shooter, 1);
  private final ShooterCommand stopShooter = new ShooterCommand(m_shooter, 0);
  private final ShootWait waitshoot = new ShootWait(m_shooter, m_intake, m_pivot);
  
  // intake commands
  private final SetIntakeCommand runIntake = new SetIntakeCommand(m_intake, 0.9);
  private final BackwardIntake backwardIntake = new BackwardIntake(m_intake, 0.8);
  //pivot commands
  private final SetPivotCommand setpivot = new SetPivotCommand(m_pivot, 0.9);
  private final PivotOtherway otherwaypivot = new PivotOtherway(m_pivot, 0.9);
  private final PivotToAngle pivottoangle60 = new PivotToAngle(m_pivot, 58); 
  private final PivotToAngle pivottoangle35 = new PivotToAngle(m_pivot, 35);
  private final PivotToAngle pivottoangle30 = new PivotToAngle(m_pivot, 30);

  private final ClimberCommand setClimberUpSpeed = new ClimberCommand(m_climber, 0.9);
  private final ClimberCommand setClimberDownSpeed = new ClimberCommand(m_climber, -0.9);

  private final AmpCommand setAmpCommand = new AmpCommand(m_pivot, 0);
  


  private final AutoAim autoaim = new AutoAim(drivebase, m_vision, m_pivot,   () -> Math.cbrt(MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                       OperatorConstants.LEFT_Y_DEADBAND) * -0.8),
                                                          () -> Math.cbrt(MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                       OperatorConstants.LEFT_X_DEADBAND) * -0.8));

  //makes the auto chooser
  private SendableChooser<String> autoChooser = new SendableChooser<String>();
  private ShuffleboardTab tab = Shuffleboard.getTab("auto chooser");
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    //commands for the autos
    NamedCommands.registerCommand("Pivot to 60", new PivotToAngle(m_pivot, 58));
    NamedCommands.registerCommand("Shoot", new ShootWaitAuto(m_shooter, m_intake, 1));
    NamedCommands.registerCommand("Intake", new SetIntakeCommandAuto( m_intake, 0.8));
    NamedCommands.registerCommand("Calib Climber", new ClimberCommand(m_climber, -0.2));
    NamedCommands.registerCommand("Auto Aim", new AutoAimInAuto(drivebase, m_vision, m_pivot));
    NamedCommands.registerCommand("Spin Up", new SpinUpAuto(m_shooter, 1));

    // add auto chooser options
    autoChooser.setDefaultOption("4 Piece", "4 Piece");
    autoChooser.addOption("Middle Speaker 2 piece", "Middle Speaker 2 piece");
    autoChooser.addOption("Left of Speaker 2 piece", "Left of Speaker 2 piece");
    autoChooser.addOption("Right of Speaker 2 piece", "Right of Speaker 2 piece");
    autoChooser.addOption("Left of Speaker get far note", "Left of Speaker get far note");
    autoChooser.addOption("Right of Speaker Wait then Shoot Auto", "Right of Speaker Wait then Shoot Auto");
    autoChooser.addOption("Left of Speaker Wait then Shoot Auto", "Left of Speaker Wait then Shoot Auto");
    autoChooser.addOption("Right of Speaker get far note", "Right of Speaker get far note");
    autoChooser.addOption("Middle Speaker 3 Piece Left", "Middle Speaker 3 Piece Left");
    autoChooser.addOption("Right Speaker Shoot Pre-Loaded", "Right Speaker Shoot Pre-Loaded");
    autoChooser.addOption("Left Speaker Shoot Pre-Loaded", "Left Speaker Shoot Pre-Loaded");
    autoChooser.addOption("Middle Speaker 3 Piece Right", "Middle Speaker 3 Piece Right");
    autoChooser.addOption("3 piece preload and two in mid field", "3 piece preload and two in mid field");
    autoChooser.addOption("4 Piece Fast", "4 Piece Fast");
    autoChooser.addOption("5 Piece", "5 Piece");
    autoChooser.addOption("Line to Line", "Line to Line");


    tab.add(autoChooser);
    configureBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                                                          // Applies deadbands and inverts controls because joysticks
                                                          // are back-right positive while robot
                                                          // controls are front-left positive
                                                          () -> Math.cbrt(MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                       OperatorConstants.LEFT_Y_DEADBAND) * -0.8),
                                                          () -> Math.cbrt(MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                       OperatorConstants.LEFT_X_DEADBAND) * -0.8),
                                                          () -> -driverXbox.getRightX(),
                                                          () -> -driverXbox.getRightY());

    AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
                                                                         () ->
                                                                             MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                    OperatorConstants.LEFT_Y_DEADBAND),
                                                                         () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                      OperatorConstants.LEFT_X_DEADBAND),
                                                                         () -> driverXbox.getRawAxis(2));

    

    
   

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDrive : closedFieldAbsoluteDrive);
    //m_pivot.setDefaultCommand(new PivotToAngle(m_pivot, 60)); TODO GET BETTER
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   * 
   */
  
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
   // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
    
   
  // driverControllerButtonB.whileTrue(autoaim); //B Button
    driverControllerleftbumper.whileTrue(runIntake); //left bumper
    driverControllerrightbumper.whileTrue(backwardIntake);
    
    
    
   // secondriverButtonA.whileTrue(pivottoangle30);
   // secondriverButtonB.whileTrue(pivottoangle35); //B button
    secondriverButtonY.onTrue(pivottoangle60); //Y button
   // secondriverButtonA.whileTrue(setpivot);
    secondriverButtonB.whileTrue(otherwaypivot);
    secondriverButtonA.whileTrue(setAmpCommand);
    secondriverButtonX.whileTrue(autoaim);
    
    secondriverButton7.whileTrue(setClimberUpSpeed);
    secondriverButton8.whileTrue(setClimberDownSpeed);
    secondriverleftbumper.whileTrue(waitshoot); //left bumper
    secondriverrightbumper.whileTrue(runIntake); //right bumper
    //secondriverButtonA.whileTrue(runShooter); //A button

    
}

  public void setLeds() {
    Color orange = new Color(200, 20, 0);
    Color blue = new Color(0, 200, 50);
    if(m_intake.hasNote()) {
      m_leds.setAll(orange);
    } else {
      m_leds.setAll(blue);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return new PathPlannerAuto(autoChooser.getSelected());
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void resetrobot() {
    m_shooter.stopShooter();
    m_intake.stopIntake();
    m_intake.stopIndexer();
  }
}