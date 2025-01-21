package frc.robot.commands.shooter;

import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootWait extends Command {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;

    private final static double leftTargetRPM = 5500;
    private final static double rightTaretRPM = 5000;

    public ShootWait(ShooterSubsystem shooter, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.intake = intake;

        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.setRightMotorRPM(rightTaretRPM);
        shooter.setLeftMotorRPM(leftTargetRPM);

        if (shooter.getMinVelocity() > (leftTargetRPM * 0.9)) {
            intake.setIntake(1);
        }

    }

    @Override
    public void end(boolean interuppted) {
        shooter.stopShooter();
        intake.setIntake(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
