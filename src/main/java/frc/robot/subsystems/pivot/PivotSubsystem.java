package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class PivotSubsystem extends SubsystemBase {

    public enum PivotLocations {
        DEG_60 (59),
        DEG_45 (45),
        DEG_30 (30);

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
    private final ProfiledPIDController pivotVoltagePID;
    private final ArmFeedforward pivotFeedforward;
    private final RelativeEncoder pivotRelativeEncoder;
    private final SparkMaxConfig pivotConfig;

    private final EventLoop m_loop = new EventLoop();

    private double targetPivotAngle = PivotLocations.DEG_60.getCommandedAngle();

    public PivotSubsystem() {

        pivotMotor = new SparkMax(Constants.PivotConstants.pivotMotorID, MotorType.kBrushless);
        pivotConfig =  new SparkMaxConfig();
        pivotConfig.inverted(false).idleMode(IdleMode.kBrake);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotEncoder = new DutyCycleEncoder(9);

        pivotFeedforward = new ArmFeedforward(0.25, 0.05, 0.08);
        pivotVoltagePID = new ProfiledPIDController(0.16, 0, 0.00095,
            new TrapezoidProfile.Constraints(70.0, 150), 0.02);

        pivotVoltagePID.setGoal(PivotLocations.DEG_60.commandedAngle);

        pivotMotor.set(0);
        pivotRelativeEncoder = pivotMotor.getEncoder();
    }

    public void setBrake(boolean brake) {
        pivotConfig.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void stopPivot() {
        pivotMotor.set(0);
    }

    @Override
    public void periodic() {
        m_loop.poll();
        SmartDashboard.putNumber("Pivot Angle", getPivotAngle());

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

        if (DriverStation.isTest()) {
            return;
        }


        double voltage = pivotVoltagePID.calculate(getPivotAngle()) + pivotFeedforward.calculate(pivotVoltagePID.getSetpoint().position, pivotVoltagePID.getSetpoint().velocity);
        voltage = Math.max(-7, Math.min(7, voltage));

        // // Disable power when close enough
        // if (!isPivotBraked && Math.abs(getPivotAngle() - targetPivotAngle) < 5) {
        //     isPivotBraked = true;
        // } else if (isPivotBraked && Math.abs(getPivotAngle() - targetPivotAngle) > 8) {
        //     isPivotBraked = false;
        // }

        // if (isPivotBraked) {
        //     voltage = 0;
        // }

        if (Math.abs(voltage) < 0.2) {
            voltage = 0;
        }

        setVoltage(Volts.of(voltage));

        SmartDashboard.putNumber("Pivot Voltage", voltage);
    }

    public void setTargetPivotAngle(PivotLocations location) {
        setTargetPivotAngle(location.getCommandedAngle());
    }

    public void setTargetPivotAngle(double angle) {
        pivotVoltagePID.setGoal(angle);
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
        Velocity<VoltageUnit> rampVelocity = Volts.of(0.5).div(Seconds.of(1.0));
        
        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(rampVelocity, Volts.of(7), Seconds.of(10)),
            new SysIdRoutine.Mechanism(this::setVoltage, this::logMotor, this)
        );

        BooleanEvent atTop = new BooleanEvent(m_loop, this::isTopEndStop);
        BooleanEvent atBottom = new BooleanEvent(m_loop, this::isBottomEndStop);

        switch (index) {
            case 0: return routine.quasistatic(SysIdRoutine.Direction.kForward).until(atTop).withTimeout(5.0);
            case 1: return routine.quasistatic(SysIdRoutine.Direction.kReverse).until(atBottom).withTimeout(5.0);
            case 2: return routine.dynamic(SysIdRoutine.Direction.kForward).until(atTop).withTimeout(5.0);
            case 3: return routine.dynamic(SysIdRoutine.Direction.kReverse).until(atBottom).withTimeout(5.0);
        }
        return routine.quasistatic(SysIdRoutine.Direction.kForward).until(atTop).withTimeout(5.0);
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

    public boolean isTopEndStop() {
        return getPivotAngle() > 70;
    }

    public boolean isBottomEndStop() {
        return getPivotAngle() < 25;
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
