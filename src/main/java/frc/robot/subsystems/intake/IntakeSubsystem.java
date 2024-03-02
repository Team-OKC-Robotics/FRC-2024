package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    private final CANSparkMax intakemotor;
    private final DigitalInput IntakeLimitSwitch;
    private final CANSparkMax indexerMotor;
    private DataLog log;
    private DoubleLogEntry posLog;
    private DoubleLogEntry outputLog;
    private DoubleLogEntry setpointLog;

    private int direction = 0;

    // shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("intake");

   

    //sensors 
    private GenericEntry intakeSwitch = tab.add("intake switch", false).getEntry();

public IntakeSubsystem() {
    intakemotor = new CANSparkMax(Constants.IntakeConstants.intakemotorID, CANSparkLowLevel.MotorType.kBrushless);
    intakemotor.restoreFactoryDefaults();
    intakemotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    intakemotor.setInverted(true);
    IntakeLimitSwitch = new DigitalInput(8);
    indexerMotor = new CANSparkMax(Constants.ShooterConstants.indexerMotorID, CANSparkLowLevel.MotorType.kBrushless);
    indexerMotor.restoreFactoryDefaults();

    log = DataLogManager.getLog();
        posLog = new DoubleLogEntry(log, "/intake/pos");
        outputLog = new DoubleLogEntry(log, "/intake/output");
        setpointLog = new DoubleLogEntry(log, "/intake/setpoint");
}

public void setSpeed(double power) {
    intakemotor.set(power);
}

public void stopIntake() {
    intakemotor.set(0);
}

public void indexerSpeed(double power) {
    indexerMotor.set(power);
  }
  
public void stopIndexer(double speed) {
    indexerMotor.set(0);
  }

public enum IntakeState {

    FORWARD(1),
    OFF(0);
    

   public double power;

   private IntakeState(double power){
       this.power = power;
   }

}

public double getSpeed() {
    return intakemotor.get();
}



public Command runIntake(double Speed){
    return run(() -> {
        setSpeed(Speed);
    });
}
@Override
public void periodic() {

    posLog.append(intakemotor.get());
    outputLog.append(intakemotor.get());

    intakeSwitch.setBoolean(IntakeLimitSwitch.get());
    
    }

public void SetIntake(double speed) {
    intakemotor.set(speed);
    indexerMotor.set(speed);
} 

public boolean hasNote() {
    return !IntakeLimitSwitch.get();
}


}
