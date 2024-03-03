// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.pivot.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.superstructure.SuperState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.commands.shooter.*;
import frc.robot.commands.vision.AutoAim;
import frc.robot.commands.pivot.*;
import frc.robot.commands.intake.*;
import java.io.File;
import java.util.Set;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.commands.intake.SetIntakeCommand;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/swerve"));

  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final PivotSubsystem m_pivot = new PivotSubsystem();
  public final Superstructure superstructure = new Superstructure(m_intake, m_shooter, m_pivot, drivebase);

  //controllers
  XboxController driverXbox = new XboxController(0);
 
  private final Joystick driverController = new Joystick(0);
  private final Joystick secondriver = new Joystick(1);
 
  //driver buttons

  private final JoystickButton driverControllerButtonB = new JoystickButton(driverController, Constants.OI.kdriverControllerButton2);
  private final JoystickButton driverControllerleftbumper = new JoystickButton(driverController, Constants.OI.kdriverControllerButton5);

  
  //second driver buttons
  private final JoystickButton secondriverButtonB = new JoystickButton(secondriver, Constants.OI.kSecondriverButton2);
  private final JoystickButton secondriverButtonY = new JoystickButton(secondriver, Constants.OI.kSecondriverButton4);
  private final JoystickButton secondriverButtonA = new JoystickButton(secondriver, Constants.OI.kSecondriverButton1);
  private final JoystickButton secondriverButtonX = new JoystickButton(secondriver, Constants.OI.kSecondriverButton3);
  private final JoystickButton secondriverleftbumper = new JoystickButton(secondriver, Constants.OI.kSecondriverButton5);
  private final JoystickButton secondriverrightbumper = new JoystickButton(secondriver, Constants.OI.kSecondriverButton6);
  //commands
  private final ShooterCommand runShooter = new ShooterCommand(m_shooter, 1);
  private final ShooterCommand stopShooter = new ShooterCommand(m_shooter, 0);
  private final ShootWait waitshoot = new ShootWait(m_shooter, m_intake, 1);

  private final SetIntakeCommand runIntake = new SetIntakeCommand(m_intake, 0.6);
  private final BackwardIntake backwardIntake = new BackwardIntake(m_intake, 0.8);
 // private final SetIntakeCommand runIntakeOtherway - new SetIntakeCommand(m_intake, -0.6);

  private final SetPivotCommand setpivot = new SetPivotCommand(m_pivot, 0.9);
  private final PivotOtherway otherwaypivot = new PivotOtherway(m_pivot, 0.9);
  private final PivotToAngle pivottoangle60 = new PivotToAngle(m_pivot, 30); 
  private final PivotToAngle pivottoangle30 = new PivotToAngle(m_pivot, 30);


  private final AutoAim autoaim = new AutoAim(drivebase, m_vision);

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    
    NamedCommands.registerCommand("Pivot to 60", new PivotToAngle(m_pivot, 50));
    NamedCommands.registerCommand("Shoot", new ShootWait(m_shooter, m_intake, 1));
    NamedCommands.registerCommand("Intake", new SetIntakeCommandAuto( m_intake, 0.6));
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                                                          // Applies deadbands and inverts controls because joysticks
                                                          // are back-right positive while robot
                                                          // controls are front-left positive
                                                          () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                       OperatorConstants.LEFT_Y_DEADBAND) * -0.8,
                                                          () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                       OperatorConstants.LEFT_X_DEADBAND) * -0.8,
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
    new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
    
   // secondriverButton5.whileTrue(superstructure.toState(SuperState.INTAKE_NOTE));
    driverControllerButtonB.whileTrue(autoaim); //B Button
    driverControllerleftbumper.whileTrue(runIntake); //left bumper
    
    
    
    secondriverButtonX.whileTrue(setpivot);//x button
    secondriverButtonB.whileTrue(pivottoangle30); //B button
    secondriverButtonY.onTrue(pivottoangle60); //Y button
    secondriverleftbumper.whileTrue(waitshoot); //left bumper
    secondriverrightbumper.whileTrue(backwardIntake); //right bumper
    secondriverButtonA.whileTrue(otherwaypivot);
    



   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return new PathPlannerAuto("Middle Speaker 2 piece");
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}