// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Optional;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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


import frc.robot.commands.pivot.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.swervedrive.drivebase.*;
import frc.robot.commands.vision.*;



import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.leds.*;
import frc.robot.subsystems.pivot.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.subsystems.vision.*;


import frc.robot.utils.POVButton;
import frc.robot.utils.TriggerButton;
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
  private final JoystickButton driverXboxButtonB = new JoystickButton(driverXbox, Constants.OI.kdriverControllerButton2);
  private final JoystickButton driverXboxleftbumper = new JoystickButton(driverXbox, Constants.OI.kdriverControllerButton5);
  private final JoystickButton driverXboxrightbumper = new JoystickButton(driverXbox, Constants.OI.kdriverControllerButton6);
  private final TriggerButton driverXboxLeftTrigger = new TriggerButton(driverXbox, 2, 0.8);
  private final TriggerButton driverXboxRightTrigger = new TriggerButton(driverXbox, 3, 0.8);
  private final JoystickButton driverXboxButtonA = new JoystickButton(driverXbox, Constants.OI.kdriverControllerButton1);
  private final JoystickButton driverXboxButtonY = new JoystickButton(driverXbox, Constants.OI.kdriverControllerButton4);
  private final JoystickButton driverXboxButtonX = new JoystickButton(driverXbox, Constants.OI.kdriverControllerButton3);
  private final JoystickButton driverXboxButtonMinus = new JoystickButton(driverXbox, Constants.OI.kdriverControllerButton7);
  private final JoystickButton driverXboxButtonPlus = new JoystickButton(driverXbox, Constants.OI.kdriverControllerButton8);
  private final POVButton driverXboxDpad = new POVButton(driverXbox, 0);
  
  //second driver buttons
  private final JoystickButton secondriverXboxButtonB = new JoystickButton(secondriverXbox, Constants.OI.kSecondriverButton2);
  private final JoystickButton secondriverXboxButtonY = new JoystickButton(secondriverXbox, Constants.OI.kSecondriverButton4);
  private final JoystickButton secondriverXboxButtonA = new JoystickButton(secondriverXbox, Constants.OI.kSecondriverButton1);
  private final JoystickButton secondriverXboxButtonX = new JoystickButton(secondriverXbox, Constants.OI.kSecondriverButton3);
  private final JoystickButton secondriverXboxleftbumper = new JoystickButton(secondriverXbox, Constants.OI.kSecondriverButton5);
  private final JoystickButton secondriverXboxrightbumper = new JoystickButton(secondriverXbox, Constants.OI.kSecondriverButton6);
  private final JoystickButton secondriverXboxButtonMinus = new JoystickButton(secondriverXbox, Constants.OI.kSecondriverButton7);
  private final JoystickButton secondriverXboxButtonPlus = new JoystickButton(secondriverXbox, Constants.OI.kSecondriverButton8);
  private final POVButton secondriverXboxDpad = new POVButton(secondriverXbox, 0);
  private final TriggerButton secondriverXboxRightTrigger = new TriggerButton(secondriverXbox, 3, 0.8);
  private final TriggerButton secondriverXboxLeftTrigger = new TriggerButton(secondriverXbox,2, 0.8);

  Color orange = new Color(255, 43, 0);
  Color cyan = new Color(0, 200, 50);
  Color green = new Color(0, 153, 0);
  Color red = new Color (200, 0, 0);
  Color blue = new Color (0, 0, 200);
  Color pink = new Color(255, 0, 128);
  
  
  // shooter commands
  private final ShooterCommand runShooter = new ShooterCommand(m_shooter, 1);
  // private final ShooterCommand stopShooter = new ShooterCommand(m_shooter, 0);
  private final ShootWait waitshoot = new ShootWait(m_shooter, m_intake, m_pivot);
  
  
  // intake commands
  private final SetIntakeCommand runIntake = new SetIntakeCommand(m_intake, 0.7);
  private final BackwardIntake backwardIntake = new BackwardIntake(m_intake, 0.5);
  //pivot commands
  // private final SetPivotCommand setpivot = new SetPivotCommand(m_pivot, 0.9);
  // private final PivotOtherway otherwaypivot = new PivotOtherway(m_pivot, 0.9);
  private final PivotToAngle pivottoangle60 = new PivotToAngle(m_pivot, 59); 
  private final PivotToAngle pivottoangle45 = new PivotToAngle(m_pivot, 44);
 

  private final ClimberCommand setClimberUpSpeed = new ClimberCommand(m_climber, 1);
  private final ClimberCommand setClimberDownSpeed = new ClimberCommand(m_climber, -1);

 
  


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
    NamedCommands.registerCommand("Pivot to 45", new PivotToAngle(m_pivot, 41));
    NamedCommands.registerCommand("Shoot", new ShootWaitAuto(m_shooter, m_intake, 1));
    NamedCommands.registerCommand("Intake", new SetIntakeCommandAuto( m_intake, 0.65));
    NamedCommands.registerCommand("Auto Aim", new AutoAimInAuto(drivebase, m_vision, m_pivot));
    NamedCommands.registerCommand("Spin Up", new SpinUpAuto(m_shooter, 1));

    // add auto chooser options
    autoChooser.setDefaultOption("4 Piece Middle First Then Amp", "4 Piece Middle First Then Amp");

    autoChooser.addOption("4 Piece Middle First Then Source", "4 Piece Middle First Then Source");

    autoChooser.addOption("4 Piece Amp First", "4 Piece Amp First");

    autoChooser.addOption("4 Piece Source First", "4 Piece Source First");

    autoChooser.addOption("Middle Speaker 2 piece", "Middle Speaker 2 piece");

    autoChooser.addOption("Amp Side Get Far Notes (no preload)", "Amp Side Get Far Notes (no preload)");

    autoChooser.addOption("Amp side 2 piece", "Amp side 2 piece");
    
    autoChooser.addOption("Amp Side Wait then Shoot Auto", "Amp Side Wait then Shoot Auto");

    autoChooser.addOption("Source Side Wait then Shoot Auto", "Source Side Wait then Shoot Auto");

    autoChooser.addOption("3 piece start against source wall", "3 piece start against source wall");

    autoChooser.addOption("Middle Speaker 3 Piece Source", "Middle Speaker 3 Piece Source");

    autoChooser.addOption("Amp Side Shoot Pre-Loaded", "Amp Side Shoot Pre-Loaded");

    autoChooser.addOption("Source Side Shoot Pre-Loaded", "Left Speaker Shoot Pre-Loaded");

    autoChooser.addOption("Middle Speaker 3 Piece Amp", "Middle Speaker 3 Piece Amp");

    autoChooser.addOption("2 piece start against amp wall", "2 piece start against amp wall");

    autoChooser.addOption("4 Piece Fast", "4 Piece Fast");

    autoChooser.addOption("2 piece start against source wall", "2 piece start against source wall");

    autoChooser.addOption("2.5 piece start against source wall far notes", "2.5 piece start against source wall far notes");

    autoChooser.addOption("Source wall get mid far notes 1.5", "Source wall get mid far notes 1.5");

    autoChooser.addOption("Offset Amp Side 4 Piece", "Offset Amp Side 4 Piece");


    tab.add(autoChooser);
    configureBindings();

  AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase, driverXbox);
    
  drivebase.setDefaultCommand(closedAbsoluteDrive);
    
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


    driverXboxleftbumper.whileTrue(runIntake); //left bumper
    driverXboxrightbumper.whileTrue(backwardIntake);
    driverXboxLeftTrigger.whileTrue(waitshoot);
    driverXboxRightTrigger.whileTrue(autoaim);
    driverXboxButtonX.whileTrue(pivottoangle60);
    driverXboxButtonY.whileTrue(pivottoangle45);
    driverXboxDpad.whileTrue(runShooter);
    driverXboxButtonMinus.whileTrue(setClimberDownSpeed);
    driverXboxButtonPlus.whileTrue(setClimberUpSpeed);
    
    //second driver commands 
    
   // secondriverXboxButtonA.whileTrue(pivottoangle30);
   // secondriverXboxButtonB.whileTrue(pivottoangle35); 
    // secondriverXboxButtonY.onTrue(pivottoangle60); //button X
   
    //secondriverXboxButtonA.whileTrue(setAmpCommand); //button B
    // secondriverXboxButtonB.whileTrue(autoaim); //button A
    // secondriverXboxButtonX.whileTrue(pivottoangle45); //button Y
    
    // secondriverXboxButtonPlus.whileTrue(setClimberUpSpeed);
    // secondriverXboxButtonMinus.whileTrue(setClimberDownSpeed);
    // secondriverXboxleftbumper.whileTrue(waitshoot); //left bumper
    // secondriverXboxrightbumper.whileTrue(runIntake); //right bumper
    // secondriverXboxDpad.whileTrue(runShooter);
    // secondriverXboxRightTrigger.whileTrue(backwardIntake);
   
}
  // makes led settings
  
  public void setLeds() {
    
    
  

    if (secondriverXbox.getBButton() == true) {
      m_leds.setAll(green);
      return;
    } 

     if(m_intake.hasNote()) {
      m_leds.setAll(orange);
    } else {
      m_leds.setAll(cyan);
    }
    

}

  public void setLEDsAlliance(){
    

    Optional <Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if(ally.get() == Alliance.Red) {
        m_leds.setAll(red);
      } else  {
          m_leds.setAll(blue);
        }
      }
    
    }
  

  public void setLEDsAuto() {
    
    //Color teal = new Color(36, 225, 212);
    if(m_intake.hasNote()) {
      m_leds.setAll(pink);
    } else {
      m_leds.rainbow();
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

  public void resetPivotPID() {
    m_pivot.resetPID();
  }
}
