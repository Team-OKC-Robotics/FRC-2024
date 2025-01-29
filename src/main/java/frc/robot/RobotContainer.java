// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.pivot.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.vision.*;

import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.leds.*;
import frc.robot.subsystems.leds.LEDSubsystem.LEDState;
import frc.robot.subsystems.pivot.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.subsystems.vision.*;
import swervelib.SwerveInputStream;
import frc.robot.commands.climber.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/swerve"));

  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final PivotSubsystem m_pivot = new PivotSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final LEDSubsystem m_leds = new LEDSubsystem();

  // controllers
  CommandXboxController driverXbox = new CommandXboxController(0);
  CommandXboxController operatorXbox = new CommandXboxController(1);

  // drive commands
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .allianceRelativeControl(true)
      .scaleTranslation(0.8);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(() -> -driverXbox.getRightX(),
          () -> -driverXbox.getRightY())
      .headingWhile(true);
      
  // shooter commands
  private final ShooterCommand runShooter = new ShooterCommand(m_shooter);
  private final ShootWait waitshoot = new ShootWait(m_shooter, m_intake);

  // intake commands
  private final SetIntakeCommand runIntake = new SetIntakeCommand(m_intake, 0.7);
  private final BackwardIntake backwardIntake = new BackwardIntake(m_intake);
  private final PivotToAngle pivotToDeg60 = new PivotToAngle(m_pivot, PivotSubsystem.PivotLocations.DEG_60);
  private final PivotToAngle pivotToDeg45 = new PivotToAngle(m_pivot, PivotSubsystem.PivotLocations.DEG_45);
  private final PivotToAngle pivotToDeg30 = new PivotToAngle(m_pivot, PivotSubsystem.PivotLocations.DEG_30);

  private final ClimberCommand setClimberUpSpeed = new ClimberCommand(m_climber, 1);
  private final ClimberCommand setClimberDownSpeed = new ClimberCommand(m_climber, -1);

  private final AutoAim autoaim = new AutoAim(drivebase, m_vision, m_pivot, m_leds, driveAngularVelocity);

  // makes the auto chooser
  private SendableChooser<String> autoChooser = new SendableChooser<String>();
  private ShuffleboardTab tab = Shuffleboard.getTab("auto chooser");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // commands for the autos
    NamedCommands.registerCommand("Pivot to 60", new PivotToAngle(m_pivot, 58));
    NamedCommands.registerCommand("Pivot to 45", new PivotToAngle(m_pivot, 41));
    NamedCommands.registerCommand("Shoot", new ShootWaitAuto(m_shooter, m_intake));
    NamedCommands.registerCommand("Intake", new SetIntakeCommandAuto(m_intake, 0.65));
    NamedCommands.registerCommand("Auto Aim", new AutoAimInAuto(m_vision, m_pivot));
    NamedCommands.registerCommand("Spin Up", new SpinUpAuto(m_shooter));

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
    autoChooser.addOption("Source Side Shoot Pre-Loaded Then Move", "Left Speaker Shoot Pre-Loaded Then Move");
    autoChooser.addOption("Middle Speaker 3 Piece Amp", "Middle Speaker 3 Piece Amp");
    autoChooser.addOption("2 piece start against amp wall", "2 piece start against amp wall");
    autoChooser.addOption("4 Piece Fast", "4 Piece Fast");
    autoChooser.addOption("2 piece start against source wall", "2 piece start against source wall");
    autoChooser.addOption("2.5 piece start against source wall far notes",
        "2.5 piece start against source wall far notes");
    autoChooser.addOption("Source wall get mid far notes 1.5", "Source wall get mid far notes 1.5");
    autoChooser.addOption("Offset Amp Side 4 Piece", "Offset Amp Side 4 Piece");

    tab.add(autoChooser);
  }

  public void configureBindings() {

    // Set LEDs to be the team/note color by default
    m_leds.setDefaultCommand(Commands.run(() -> {
      if (m_intake.hasNote()) {
        m_leds.setLEDState(LEDState.HAS_NOTE);
      } else {
        m_leds.setLEDState(LEDState.TEAM);
      }
    }, m_leds));

    // Pivot to 60 when the robot is doing nothing else
    m_pivot.setDefaultCommand(pivotToDeg60);

    if (!DriverStation.isTest()) {
      drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveDirectAngle));

      // driver commands
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().whileTrue(pivotToDeg60);
      driverXbox.y().whileTrue(pivotToDeg30);
      driverXbox.povCenter().whileTrue(runShooter);
      // driverXbox.back().whileTrue(setClimberDownSpeed);
      // driverXbox.start().whileTrue(setClimberUpSpeed);
      driverXbox.leftBumper().whileTrue(runIntake);
      driverXbox.rightBumper().whileTrue(backwardIntake);
      driverXbox.leftTrigger().whileTrue(waitshoot);
      driverXbox.rightTrigger().whileTrue(autoaim);
      driverXbox.start().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(13.9 + 0.9, 4.026), Rotation2d.fromDegrees(180)))
                              );

      // // operator commands
      // operatorXbox.y().whileTrue(pivotToDeg60);
      // operatorXbox.b().whileTrue(autoaim);
      // operatorXbox.x().whileTrue(pivotToDeg45);
      // operatorXbox.leftBumper().whileTrue(waitshoot);
      // operatorXbox.rightBumper().whileTrue(runIntake);
      // operatorXbox.povCenter().whileTrue(runShooter);
      // operatorXbox.leftTrigger().whileTrue(waitshoot);
      // operatorXbox.rightTrigger().whileTrue(backwardIntake);
    } else {
      drivebase.removeDefaultCommand();

      driverXbox.a().whileTrue(m_pivot.sysIdPivotMotor(0));
      driverXbox.b().whileTrue(m_pivot.sysIdPivotMotor(1));
      driverXbox.y().whileTrue(m_pivot.sysIdPivotMotor(2));
      driverXbox.x().whileTrue(m_pivot.sysIdPivotMotor(3));

      driverXbox.start().whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.back().whileTrue(drivebase.sysIdAngleMotorCommand());
    }
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto(autoChooser.getSelected());
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void setPivotBrake(boolean brake) {
    m_pivot.setBrake(brake);
  }

  public void resetRobot() {
    m_shooter.stopShooter();
    m_intake.stopIntake();
    m_intake.stopIndexer();
  }

  public void periodic5ms() {
    m_intake.stopIntakePeriodic();
  }
}
