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

    private final SparkMax pivotmotor;
    private final SparkMax ampdevicemotor;
    private final PIDController AmpPidController;
    private final PIDController PivotPIDController;
    private final DutyCycleEncoder AmpEncoder;
    private final DutyCycleEncoder pivotencoder;

    private final PIDController pivotVoltagePID;
    private final ArmFeedforward pivotFeedforward;

    private final RelativeEncoder pivotRelativeEncoder;

    private double targetAmpangle = 24;
    private double targetPivotangle = 59;

    private State currentState = State.AMP_IN;
    private State targetState = State.AMP_IN;

    private ShuffleboardTab pivottab = Shuffleboard.getTab("pivot");
    // private ShuffleboardTab amptab = Shuffleboard.getTab("amp");
    private ShuffleboardTab comptab = Shuffleboard.getTab("comp");

    private GenericEntry AmpDeviceEncoder = comptab.add("amp encoder", 0).getEntry();

    private GenericEntry pivotabsoluteencoder = pivottab.add("absolute encoder", 0).getEntry();
    private GenericEntry pivotabsoluteencoderraw = pivottab.add("absolute encoder raw", 0).getEntry();
    private GenericEntry pivotvoltageout = pivottab.add("voltage", 0).getEntry();


    // private GenericEntry CurrentState = comptab.add("Current State", "Amp
    // In").getEntry();

    private GenericEntry TargetEncoder = pivottab.add("target encoder", 60.0).getEntry();

    private GenericEntry EncoderButton = pivottab.add("Set Encoder", false).getEntry();

    SparkMaxConfig pivotconfig = new SparkMaxConfig();
    SparkMaxConfig ampconfig = new SparkMaxConfig();

    // private GenericEntry motorpower = pivottab.add("motor power",
    // 0.0).getEntry();

    public PivotSubsystem() {

        pivotmotor = new SparkMax(Constants.PivotConstants.pivotmotorID, MotorType.kBrushless);
        ampdevicemotor = new SparkMax(Constants.AmpConstants.ampdevicemotorID, MotorType.kBrushless);

        pivotconfig.inverted(false).idleMode(IdleMode.kBrake);
        ampconfig.inverted(true).idleMode(IdleMode.kCoast);

        pivotmotor.configure(pivotconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        ampdevicemotor.configure(ampconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotencoder = new DutyCycleEncoder(9); // TODO: Add extra parameters https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html#configuring-duty-cycle-encoder-range-and-zero
        AmpEncoder = new DutyCycleEncoder(0);

        PivotPIDController = new PIDController(0.05, 0.001, 0);
        AmpPidController = new PIDController(0.04, 0.0001, 0);

        pivotFeedforward = new ArmFeedforward(0.23525, 0.040443, 0.0, 0.0);
        pivotVoltagePID = new PIDController(0.19142, 0.0, 0.0018289);

        pivotmotor.set(0);
        ampdevicemotor.set(0);

        pivotRelativeEncoder = pivotmotor.getEncoder();
    }

    public enum State {
        AMP_IN,
        AMP_IN_PIVOT_MOVING,
        AMP_MOVING_PIVOT_OUT,
        AMP_OUT_PIVOT_MOVING,
        AMP_ENGAGED;
    }

    public void changeAngle(double power) {
        pivotmotor.set(power);
    }

    public void stopPivot() {
        pivotmotor.set(0);
    }

    public double getAngle() {
        return pivotencoder.get() * 360; // TODO: This was changed, is it right?
    }

    public void AmpDeviceOut(double power) {
        ampdevicemotor.set(power);
    }

    public void stopAmp(double power) {
        ampdevicemotor.set(0);
    }

    @Override
    public void periodic() {
        pivotabsoluteencoder.setDouble(getPivotAngle());
        pivotabsoluteencoderraw.setDouble(pivotencoder.get());
        AmpDeviceEncoder.setDouble(getDevicePosition());
        
        // TargetEncoder.setDouble(targetPivotangle);
        if (EncoderButton.getBoolean(false)) {
            EncoderButton.setBoolean(false);
            targetPivotangle = TargetEncoder.getDouble(45);
        }

        // CurrentState.setString(currentState.name());

        // TODO: Enable later
        if (!DriverStation.isTest()) {
            PivotIttoAngle(targetPivotangle);
        }

        // PivotAmpToAngle(targetAmpangle);

        // if (currentState == State.AMP_IN && targetState == State.AMP_ENGAGED) {
        //     currentState = State.AMP_IN_PIVOT_MOVING;
        // }

        // if (currentState == State.AMP_IN_PIVOT_MOVING && targetState == State.AMP_ENGAGED) { // if the amp is in and the
        //                                                                                      // pivot is moving and the
        //                                                                                      // target state is amp
        //                                                                                      // engaged
        //     targetPivotangle = 25; // move pivot to 25
        //     if (getPivotAngle() < 27) { // once pivot is less than 27 degrees
        //         currentState = State.AMP_MOVING_PIVOT_OUT; // the state is now amp moving pivot out
        //     }
        // }

        // if (currentState == State.AMP_MOVING_PIVOT_OUT && targetState == State.AMP_ENGAGED) {
        //     targetAmpangle = -22.5;
        //     if (getDevicePosition() < -20) {
        //         currentState = State.AMP_OUT_PIVOT_MOVING;
        //     }
        // }

        // if (currentState == State.AMP_OUT_PIVOT_MOVING && targetState == State.AMP_ENGAGED) {
        //     targetPivotangle = 53;
        //     if (getPivotAngle() > 52) {
        //         currentState = State.AMP_ENGAGED;
        //     }
        // }

        // if (currentState == State.AMP_ENGAGED && targetState == State.AMP_IN) {
        //     currentState = State.AMP_OUT_PIVOT_MOVING;
        // }

        // if (currentState == State.AMP_OUT_PIVOT_MOVING && targetState == State.AMP_IN) { // if the amp is in and the
        //                                                                                  // pivot is moving and the
        //                                                                                  // target state is amp engaged
        //     targetPivotangle = 25; // move pivot to 30
        //     if (getPivotAngle() < 27) { // once pivot is less than 35 degrees
        //         currentState = State.AMP_MOVING_PIVOT_OUT; // the state is now amp moving pivot out
        //     }
        // }

        // if (currentState == State.AMP_MOVING_PIVOT_OUT && targetState == State.AMP_IN) {
        //     targetAmpangle = 24;
        //     if (getDevicePosition() > 22) {
        //         currentState = State.AMP_IN_PIVOT_MOVING;
        //     }
        // }

        // if (currentState == State.AMP_IN_PIVOT_MOVING && targetState == State.AMP_IN) {
        //     targetPivotangle = 57;
        //     if (getPivotAngle() > 53) {
        //         currentState = State.AMP_IN;
        //     }
        // }

    }

    public void PivotIt(double power) {
        // motorpower.setDouble(power);
        // adds soft limits to avoid the pivot killing itself
        if (power > 0 && getPivotAngle() > 70) {
            pivotmotor.set(0);
            return;
        }
        if (power < 0 && getPivotAngle() < 20) {
            pivotmotor.set(0);
            return;
        }

        pivotmotor.set(power);
    }

    public void PivotIttoAngle(double angle) {
        if (Math.abs(getPivotAngle() - angle) < 0.3) {
            PivotIt(0);
            return;
        }
        // double power = PivotPIDController.calculate(getPivotAngle(), angle);
        // power = MathUtil.clamp(power, -0.4, 0.4);
        double voltage = pivotVoltagePID.calculate(getPivotAngle(), angle) + pivotFeedforward.calculate(angle, 0.0);
        // PivotIt(power);
        setVoltage(Voltage.ofBaseUnits(voltage, Volts));
    }

    public void SetTargetPivotAngle(double angle) {
        if (currentState == State.AMP_IN) {
            targetPivotangle = angle;
            PivotPIDController.reset();
        }
    }

    public double getPivotAngle() {
        double rawvalue = pivotencoder.get();
        if (rawvalue > 0.5) { // bc the absolute encoder is messed up :(
            rawvalue = rawvalue - 1;
        }
        return -rawvalue * 360 + 34.86; // some weird ahh math, I know
    }

    public void PivotAmp(double power) {
        ampdevicemotor.set(power);
    }

    public void PivotAmpToAngle(double angle) {
        PivotAmp(AmpPidController.calculate(getDevicePosition(), angle));

    }

    public void SetTargetAmpAngle(double angle) {
        targetAmpangle = angle;
    }

    public void desireAmpEngaged() {
        targetState = State.AMP_ENGAGED;
    }

    public void desireAmpIn() {
        targetState = State.AMP_IN;
    }

    public boolean IsAmpIn() {

        if (currentState == State.AMP_IN) {
            return true;
        } else {
            return false;
        }
    }

    public double getDevicePosition() {
        double rawvalue = AmpEncoder.get();
        if (rawvalue > 0.5) { // bc the absolute encoder is messed up :(
            rawvalue = rawvalue - 1;
        }
        return -rawvalue * 100; // some weird ahh math, I know
    }

    public boolean PivotAngleis60() {
        if (getPivotAngle() == 57) {
            return true;
        } else {
            return false;
        }
    }

    public boolean PivotAngleis45() {
        if (getPivotAngle() == 43) {
            return true;
        } else {
            return false;
        }
    }

    public void resetPID() {
        PivotPIDController.reset();
    }

    public boolean isPivotAtSetpoint() {
        return Math.abs(getPivotAngle() - targetPivotangle) < 3;
    }

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
        pivotvoltageout.setDouble(pivotmotor.getAppliedOutput() * pivotmotor.getBusVoltage());
        if (volts.magnitude() < 0 && getPivotAngle() < 25) {
            pivotmotor.setVoltage(0);
        } else if (volts.magnitude() > 0 && getPivotAngle() > 70) {
            pivotmotor.setVoltage(0);
        } else {
            pivotmotor.setVoltage(volts);
        }
    }

    // Mutable holder for unit-safe values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutAngle m_angle = Degrees.mutable(0);
    private final MutAngularVelocity m_velocity = DegreesPerSecond.mutable(0);

    public void logMotor(SysIdRoutineLog log) {
        log.motor("pivot")
                .voltage(m_appliedVoltage.mut_replace(pivotmotor.getAppliedOutput() * pivotmotor.getBusVoltage(), Volts))
                .angularPosition(m_angle.mut_replace(getPivotAngle(), Degrees))
                .angularVelocity(m_velocity.mut_replace(pivotRelativeEncoder.getVelocity() * 6.0, DegreesPerSecond));
    }

    public void setBrake(boolean brake) {
        pivotconfig.inverted(false).idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);

        pivotmotor.configure(pivotconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
}
