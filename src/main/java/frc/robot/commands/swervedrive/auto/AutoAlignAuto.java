// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swervedrive.drivebase.AutoAim;
import frc.robot.commands.swervedrive.drivebase.DriveCommand;
import frc.robot.commands.swervedrive.drivebase.TimedDriveCommand;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAlignAuto extends SequentialCommandGroup {
  /** Creates a new AutoAlignAuto. */
  public AutoAlignAuto(SwerveSubsystem swerve, VisionSubsystem vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TimedDriveCommand(1.0, swerve, 0.8, 0, 0, 0),
      new TimedDriveCommand(0.5, swerve, 0, 0, 0, 0),
      new AutoAim(swerve, vision),
      new WaitCommand(5),
      new TimedDriveCommand(1.0, swerve, -0.75, 0, 0, 0),
      new TimedDriveCommand(0.5, swerve, 0, 0, 0, 0)
    );
  }
}
