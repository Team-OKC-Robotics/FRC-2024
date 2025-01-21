package frc.robot.commands.shooter;

import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootWaitAuto extends Command {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;

    private final static double leftTargetRPM = 5500;
    private final static double rightTaretRPM = 5000;
    private boolean hasNoteLeft = false;

    public ShootWaitAuto(ShooterSubsystem shooter, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.intake = intake;

        addRequirements(shooter, intake);

    }

    @Override
    public void initialize() {
        hasNoteLeft = false;
    }

    @Override
    public void execute() {
        if (!intake.hasNote()) {
            hasNoteLeft = true;
            return;
        }
        shooter.setLeftMotorRPM(leftTargetRPM);
        shooter.setRightMotorRPM(rightTaretRPM);

        if (shooter.getMinVelocity() > (leftTargetRPM * 0.9)) {
            intake.setIntake(1);
        }
    }

    @Override
    public void end(boolean interuppted) {
        intake.setIntake(0);
    }

    @Override
    public boolean isFinished() {
        return hasNoteLeft;
    }
}
