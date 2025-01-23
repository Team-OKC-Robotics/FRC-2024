package frc.robot.subsystems.intake;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax intakemotor;
    private final DigitalInput IntakeLimitSwitch;
    private final SparkMax indexerMotor;

    private enum INTAKE_STATE {
        INTAKE, OUTTAKE, HOLD
    };

    private INTAKE_STATE intakeState = INTAKE_STATE.HOLD;

    // shuffleboard
    private ShuffleboardTab comptab = Shuffleboard.getTab("intake");
    // sensors
    private GenericEntry intakeSwitch = comptab.add("intake switch", false).getEntry();

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

    // sets intake speed
    public void setSpeed(double power) {
        intakemotor.set(power);
    }

    // stops intake
    public void stopIntake() {
        intakemotor.set(0);
    }

    // for backwards intake
    public void setbackSpeed(double power) {
        intakemotor.set(-power);
    }

    // for backwards index
    public void setIndexerback(double power) {
        indexerMotor.set(-power);
    }

    // sets indexer motor speed
    public void indexerSpeed(double power) {
        indexerMotor.set(power);
    }

    // stops indexer
    public void stopIndexer() {
        indexerMotor.set(0);
    }

    public double getSpeed() {
        return intakemotor.get();
    }

    public Command runIntake(double Speed) {
        return run(() -> {
            setSpeed(Speed);
        });
    }

    @Override
    public void periodic() {
        intakeSwitch.setBoolean(IntakeLimitSwitch.get());
    }

    public void setStateIntake() {
        intakeState = INTAKE_STATE.INTAKE;
    }

    public void setStateOuttake() {
        intakeState = INTAKE_STATE.OUTTAKE;
    }

    public void setStateHold() {
        intakeState = INTAKE_STATE.HOLD;
    }

    public void stopIntakePeriodic() {
        if (hasNote() && intakeState == INTAKE_STATE.INTAKE) {
            intakeState = INTAKE_STATE.HOLD;
            intakemotor.set(0);
            indexerMotor.set(0);
        }

        switch (intakeState) {
            case INTAKE:
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

    // sets intake in commmand
    public void setIntake(double speed) {
        intakemotor.set(speed);
        indexerMotor.set(speed);
    }

    // testing if the limit switch sees the note or not
    public boolean hasNote() {
        return !IntakeLimitSwitch.get();
    }

}
