package frc.robot.subsystems.intake;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax intakemotor;
    private final DigitalInput IntakeLimitSwitch;
    private final SparkMax indexerMotor;

    public final Trigger hasNote = new Trigger(this::hasNote);

    private enum INTAKE_STATE {
        INTAKE, OUTTAKE, HOLD, SHOOT
    };

    private INTAKE_STATE intakeState = INTAKE_STATE.HOLD;

    public IntakeSubsystem() {
        intakemotor = new SparkMax(Constants.IntakeConstants.intakeMotorID, MotorType.kBrushless);
        indexerMotor = new SparkMax(Constants.ShooterConstants.indexerMotorID, MotorType.kBrushless);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        SparkMaxConfig indexerConfig = new SparkMaxConfig();

        intakeConfig.inverted(true).idleMode(IdleMode.kBrake);
        indexerConfig.inverted(false).idleMode(IdleMode.kBrake);

        intakemotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        IntakeLimitSwitch = new DigitalInput(Constants.IntakeConstants.intakeLimitSwitchChannel);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Limit Switch", IntakeLimitSwitch.get());
    }

    public Command shoot() {
        return run(() -> {intakeState = INTAKE_STATE.SHOOT; setMotors();}).until(hasNote.negate());
    }

    public Command intake() {
        return run(() -> {intakeState = INTAKE_STATE.INTAKE; setMotors();}).until(hasNote);
    }

    public Command outake() {
        return run(() -> {intakeState = INTAKE_STATE.OUTTAKE; setMotors();});
    }

    public Command hold() {
        return run(() -> {intakeState = INTAKE_STATE.HOLD; setMotors();});
    }

    private void setMotors() {
        if (hasNote() && intakeState == INTAKE_STATE.INTAKE) {
            intakeState = INTAKE_STATE.HOLD;
            intakemotor.set(0);
            indexerMotor.set(0);
        }

        switch (intakeState) {
            case INTAKE, SHOOT:
                intakemotor.set(0.7);
                indexerMotor.set(0.7);
                break;
            case OUTTAKE:
                intakemotor.set(-0.5);
                indexerMotor.set(-0.5);
                break;
            default:
                intakemotor.set(0);
                indexerMotor.set(0);
                break;
        }
    }

    public boolean hasNote() {
        return !IntakeLimitSwitch.get();
    }

}
