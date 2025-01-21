package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public class PivotSubsystem extends SubsystemBase {

    public enum PivotLocations {
        DEG_60 (59),
        DEG_45 (45);

        private final double commandedAngle;
        PivotLocations(double commandedAngle) {
            this.commandedAngle = commandedAngle;
        }

        public double getCommandedAngle() {
            return commandedAngle;
        }
    }

    private final SparkMax pivotMotor;
    private final DutyCycleEncoder pivotEncoder;
    private final PIDController pivotVoltagePID;
    private final ArmFeedforward pivotFeedforward;
    private final RelativeEncoder pivotRelativeEncoder;
    private final SparkMaxConfig pivotConfig;

    private ShuffleboardTab pivotTab = Shuffleboard.getTab("pivot");
    private GenericEntry pivotAngleEntry = pivotTab.add("pivot angle", 0).getEntry();

    private double targetPivotAngle = PivotLocations.DEG_60.getCommandedAngle();

    public PivotSubsystem() {

        pivotMotor = new SparkMax(Constants.PivotConstants.pivotmotorID, MotorType.kBrushless);
        pivotConfig =  new SparkMaxConfig();
        pivotConfig.inverted(false).idleMode(IdleMode.kBrake);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotEncoder = new DutyCycleEncoder(9);

        pivotFeedforward = new ArmFeedforward(0.23525, 0.040443, 0.0, 0.0);
        pivotVoltagePID = new PIDController(0.19142, 0.0, 0.0018289);

        pivotMotor.set(0);
        pivotRelativeEncoder = pivotMotor.getEncoder();
    }

    public void setBrake(boolean brake) {
        pivotConfig.inverted(false).idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void stopPivot() {
        pivotMotor.set(0);
    }

    @Override
    public void periodic() {
        pivotAngleEntry.setDouble(getPivotAngle());

        if (!DriverStation.isTest()) {
            updatePivotLoop();
        }
    }

    public void PivotIt(double power) {
        // adds soft limits to avoid the pivot killing itself
        if (power > 0 && getPivotAngle() > 70) {
            pivotMotor.set(0);
            return;
        }
        if (power < 0 && getPivotAngle() < 20) {
            pivotMotor.set(0);
            return;
        }

        pivotMotor.set(power);
    }

    public void updatePivotLoop() {

        // Disable power when close enough
        if (Math.abs(getPivotAngle() - targetPivotAngle) < 0.3) {
            PivotIt(0);
            return;
        }

        double voltage = pivotVoltagePID.calculate(getPivotAngle(), targetPivotAngle) + pivotFeedforward.calculate(targetPivotAngle, 0.0);
        setVoltage(Voltage.ofBaseUnits(voltage, Volts));
    }

    public void setTargetPivotAngle(PivotLocations location) {
        setTargetPivotAngle(location.getCommandedAngle());
    }

    public void setTargetPivotAngle(double angle) {
        targetPivotAngle = angle;
    }

    public double getPivotAngle() {
        double rawvalue = pivotEncoder.get();
        if (rawvalue > 0.5) { // bc the absolute encoder is messed up :(
            rawvalue = rawvalue - 1;
        }
        return -rawvalue * 360 + 34.86; // some weird ahh math, I know
    }

    public boolean isPivotAtSetpoint() {
        return Math.abs(getPivotAngle() - targetPivotAngle) < 3;
    }

    // SYS ID Stuff Below

    public Command sysIdPivotMotor(int index) {
        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(this::setVoltage, this::logMotor, this)
        );

        switch (index) {
            case 0: return routine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(5.0);
            case 1: return routine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(5.0);
            case 2: return routine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(5.0);
            case 3: return routine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(5.0);
        }
        return routine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(5.0);
    }

    public void setVoltage(Voltage volts) {
        if (volts.magnitude() < 0 && getPivotAngle() < 25) {
            pivotMotor.setVoltage(0);
        } else if (volts.magnitude() > 0 && getPivotAngle() > 70) {
            pivotMotor.setVoltage(0);
        } else {
            pivotMotor.setVoltage(volts);
        }
    }

    // Mutable holder for unit-safe values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutAngle m_angle = Degrees.mutable(0);
    private final MutAngularVelocity m_velocity = DegreesPerSecond.mutable(0);

    public void logMotor(SysIdRoutineLog log) {
        log.motor("pivot")
                .voltage(m_appliedVoltage.mut_replace(pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage(), Volts))
                .angularPosition(m_angle.mut_replace(getPivotAngle(), Degrees))
                .angularVelocity(m_velocity.mut_replace(pivotRelativeEncoder.getVelocity() * 6.0, DegreesPerSecond));
    }
}
