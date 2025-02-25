package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.utils.LerpedLUT;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;

public class PivotSubsystem extends SubsystemBase {

    public enum PivotLocations {
        DEG_60(59),
        DEG_45(45),
        DEG_30(30);

        private final double commandedAngle;

        PivotLocations(double commandedAngle) {
            this.commandedAngle = commandedAngle;
        }

        public double getCommandedAngle() {
            return commandedAngle;
        }
    }

    private final LerpedLUT angleLUT;
    private final SparkMax pivotMotor;
    private final DutyCycleEncoder pivotEncoder;
    private final ProfiledPIDController pivotVoltagePID;
    private final ArmFeedforward pivotFeedforward;
    private final SparkMaxConfig pivotConfig;

    public PivotSubsystem() {

        pivotConfig = new SparkMaxConfig();
        pivotConfig.inverted(false).idleMode(IdleMode.kBrake);

        pivotMotor = new SparkMax(Constants.PivotConstants.pivotMotorID, MotorType.kBrushless);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotor.set(0);

        pivotEncoder = new DutyCycleEncoder(9);

        pivotFeedforward = new ArmFeedforward(0.25, 0.05, 0.08);
        pivotVoltagePID = new ProfiledPIDController(0.15, 0, 0.00095,
                new TrapezoidProfile.Constraints(60.0, 150), 0.02);
        pivotVoltagePID.setTolerance(Constants.PivotConstants.angleTolerance);
        pivotVoltagePID.setGoal(PivotLocations.DEG_60.commandedAngle);

        angleLUT = new LerpedLUT();

        angleLUT.addEntry(-100, 60);
        angleLUT.addEntry(0, 58); // distance in feet, angle in degrees
        angleLUT.addEntry(2.17, 43);
        angleLUT.addEntry(3.37, 38);
        angleLUT.addEntry(3.9, 35.5);
        angleLUT.addEntry(4.33, 35);
        angleLUT.addEntry(5.0, 31.5);
        angleLUT.addEntry(5.33, 32.8);
        angleLUT.addEntry(5.5, 30.6);
        angleLUT.addEntry(6.33, 29);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Angle", getPivotAngle());
    }

    public void setBrakeIdle() {
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setCoastIdle() {
        pivotConfig.idleMode(IdleMode.kCoast);
        pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command holdPosition() {
        return run(this::runPID);
    }

    public Command movetoPosition(PivotLocations location) {
        return moveToPosition(location.getCommandedAngle());
    }

    public Command moveToPosition(double angle) {
        return run(() -> {
            setTargetPivotAngle(angle);
            runPID();
        }).until(() -> pivotVoltagePID.atGoal());
    }

    public Command aimAtTarget(Vision vision) {
        return run(() -> {
            setTargetPivotAngle(angleLUT.getAngleFromDistance(Meters.of(vision.getDistanceFromAprilTag(7).orElse(100.0))));
            runPID();
        });
    }

    public Command setVoltage(Voltage voltage) {
        return run(() -> applyVoltage(voltage));
    }

    private void runPID() {
        double voltage = pivotVoltagePID.calculate(getPivotAngle()) + pivotFeedforward
                .calculate(pivotVoltagePID.getSetpoint().position, pivotVoltagePID.getSetpoint().velocity);
        voltage = Math.max(-7, Math.min(7, voltage));

        if (Math.abs(voltage) < 0.2) {
            voltage = 0;
        }

        applyVoltage(Volts.of(voltage));
    }

    private void setTargetPivotAngle(double angle) {
        pivotVoltagePID.setGoal(angle);
    }

    private double getPivotAngle() {
        double rawvalue = pivotEncoder.get();
        if (rawvalue > 0.5) { // bc the absolute encoder is messed up :(
            rawvalue = rawvalue - 1;
        }
        return -rawvalue * 360 + 34.86; // some weird ahh math, I know
    }

    private void applyVoltage(Voltage volts) {
        if (volts.magnitude() < 0 && getPivotAngle() < 25) {
            volts = Volts.of(0);
        }

        if (volts.magnitude() > 0 && getPivotAngle() > 70) {
            volts = Volts.of(0);
        }

        pivotMotor.setVoltage(volts);

        SmartDashboard.putNumber("Pivot Voltage", volts.in(Volts));
    }
}
