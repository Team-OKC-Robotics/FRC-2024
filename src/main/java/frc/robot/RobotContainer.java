// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.leds.*;
import frc.robot.subsystems.leds.LEDSubsystem.LEDState;
import frc.robot.subsystems.pivot.*;
import frc.robot.subsystems.pivot.PivotSubsystem.PivotLocations;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.swervedrive.*;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/swerve"));

  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final PivotSubsystem pivot = new PivotSubsystem();
  private final LEDSubsystem LEDs = new LEDSubsystem();
  private final Vision vision = new Vision(drivebase::getPose, drivebase.getSwerveDrive().field);

  // controllers
  CommandXboxController driverXbox = new CommandXboxController(0);
  CommandXboxController operatorXbox = new CommandXboxController(1);

  // drive commands
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .allianceRelativeControl(true)
      .scaleTranslation(0.8);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(() -> -driverXbox.getRightX(),
          () -> -driverXbox.getRightY())
      .headingWhile(true);

  // makes the auto chooser
  private SendableChooser<String> autoChooser = new SendableChooser<String>();

  public RobotContainer() {
    // commands for the autos
    NamedCommands.registerCommand("Pivot to 60", pivot.movetoPosition(PivotLocations.DEG_60));
    NamedCommands.registerCommand("Pivot to 45", pivot.movetoPosition(PivotLocations.DEG_45));
    NamedCommands.registerCommand("Shoot", intake.shoot());
    NamedCommands.registerCommand("Intake", intake.intake());
    NamedCommands.registerCommand("Spin Up", shooter.runContinously());
    NamedCommands.registerCommand("Auto Aim", pivot.aimAtTarget(vision).alongWith(drivebase.aimAtTarget(vision)));

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

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void configureBindings() {

    // Set LEDs to be the team/note color by default
    LEDs.setDefaultCommand(Commands.run(() -> {
      if (intake.hasNote()) {
        LEDs.setLEDState(LEDState.HAS_NOTE);
      } else {
        LEDs.setLEDState(LEDState.TEAM);
      }
    }, LEDs));

    drivebase.removeDefaultCommand();

    intake.setDefaultCommand(intake.hold());
    shooter.setDefaultCommand(shooter.stop());
    pivot.setDefaultCommand(pivot.holdPosition());

    if (!DriverStation.isTest()) {
      drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveDirectAngle));

      // driver commands
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().whileTrue(pivot.movetoPosition(PivotLocations.DEG_60));
      driverXbox.y().whileTrue(pivot.movetoPosition(PivotLocations.DEG_30));
      driverXbox.povCenter().whileTrue(shooter.runContinously());

      driverXbox.rightBumper().whileTrue(intake.intake());
      driverXbox.leftBumper().whileTrue(intake.outake());

      driverXbox.rightTrigger().whileTrue(shooter.spinUp()
          .andThen(shooter.runContinously())
          .alongWith(intake.shoot())
          .andThen(intake.hold()));
      driverXbox.leftTrigger().whileTrue(pivot.aimAtTarget(vision).alongWith(drivebase.aimAtTarget(vision)));
    } else {
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
    pivot.setBrakeIdle();
  }

  public void updateOdometry() {
    vision.updatePoseEstimation(drivebase.getSwerveDrive());
  }

  public void resetRobot() {
    shooter.getCurrentCommand().cancel();
    intake.getCurrentCommand().cancel();
    intake.getCurrentCommand().cancel();
  }
}
