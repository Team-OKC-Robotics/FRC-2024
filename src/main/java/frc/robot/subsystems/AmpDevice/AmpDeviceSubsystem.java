package frc.robot.subsystems.AmpDevice;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpDeviceSubsystem extends SubsystemBase{
    private final CANSparkMax ampdevicemotor;
    private final PIDController AmpPidController;
    private final DutyCycleEncoder AmpEncoder;

    private ShuffleboardTab tab = Shuffleboard.getTab("amp");
  
    private GenericEntry AmpDeviceEncoder = tab.add("amp encoder", 0).getEntry();

public AmpDeviceSubsystem() {
    ampdevicemotor = new CANSparkMax(Constants.AmpConstants.ampdevicemotorID, CANSparkLowLevel.MotorType.kBrushless);
    ampdevicemotor.restoreFactoryDefaults();

    ampdevicemotor.setInverted(false);
    AmpEncoder = new DutyCycleEncoder(0);
    ampdevicemotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    AmpPidController = new PIDController(0.03, 0.0001, 0);

    ampdevicemotor.set(0);
}

public void AmpDeviceOut(double power) {
    ampdevicemotor.set(power);
}

public void stopAmp(double power) {
    ampdevicemotor.set(0);
}

@Override 
public void periodic() {
    AmpDeviceEncoder.setDouble(getDeviceAngle());
}

public void PivotAmp(double power) {
    ampdevicemotor.set(0.4);
}

public void PivotAmpToAngle(double angle) {
    PivotAmp(AmpPidController.calculate(getDeviceAngle(), angle));
}

public double getDeviceAngle() {
    return AmpEncoder.getAbsolutePosition();
}
}
