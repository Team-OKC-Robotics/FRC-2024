package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax rightclimbmotor;
    private final CANSparkMax leftclimbmotor;
    private final DigitalInput ClimberLimitSwitch;
    private final SparkPIDController PIDController;
    

   
     
    private DataLog log;
    private DoubleLogEntry posLog;
    private DoubleLogEntry outputLog;
    private DoubleLogEntry setpointLog;
    
    
    public double target_climb;

    private ShuffleboardTab tab = Shuffleboard.getTab("climber");
    private GenericEntry climberSwitch = tab.add("climber switch", false).getEntry();
    


public ClimberSubsystem() {
    rightclimbmotor = new CANSparkMax(Constants.ClimberConstants.rightclimbermotorID, MotorType.kBrushless);
    leftclimbmotor = new CANSparkMax(Constants.ClimberConstants.leftclimbermotorID, MotorType.kBrushless);
    
    rightclimbmotor.restoreFactoryDefaults();
    leftclimbmotor.restoreFactoryDefaults();

    rightclimbmotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftclimbmotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    
    ClimberLimitSwitch = new DigitalInput(11);
    PIDController = leftclimbmotor.getPIDController();
    
    
    rightclimbmotor.setInverted(true);
    leftclimbmotor.setInverted(false);

    log = DataLogManager.getLog();
    posLog = new DoubleLogEntry(log, "/climber/pos");
    outputLog = new DoubleLogEntry(log, "/climber/output");
    setpointLog = new DoubleLogEntry(log, "/climber/setpoints");


    set(PIDF.PORPORTION, PIDF.INTEGRAL, PIDF.DERIVATIVE,
              PIDF.FEEDFORWARD, PIDF.INTEGRAL_ZONE);

}

public static class PIDF {
    /*Feedforward constant for PID loop */
    public static final double FEEDFORWARD = 0.01;
    /*Porportion constant for PID loop */
    public static final double PORPORTION = 0.05;
    /*Integral constant for PID loop */
    public static final double INTEGRAL = 0.0;
    /*Derivative constant for PID loop */
    public static final double DERIVATIVE = 0.0;
    /*Integral zone constant for PID loop */
    public static final double INTEGRAL_ZONE = 0.0;
  }



public void climbupspeed(double power) {
    rightclimbmotor.set(power);
    leftclimbmotor.set(power);

}
public void stopclimb() {
    rightclimbmotor.set(0);
    leftclimbmotor.set(0);
}

public void set(double p, double i, double d, double f, double iz) {
    PIDController.setP(p);
    PIDController.setI(i);
    PIDController.setD(d);
    PIDController.setFF(f);
    PIDController.setIZone(iz);
   }

public void runPID(double targetclimb){
    target_climb = targetclimb;
    PIDController.setReference(targetclimb, CANSparkMax.ControlType.kVelocity);
   }

public double getSpeed() {
    return rightclimbmotor.get();

}

public double getPower() {
    return rightclimbmotor.get();
}

public Command climbIt(double Speed) {
    return run(() -> runPID(Speed));
}

@Override
  public void periodic() {
    // This method will be called once per scheduler run
    posLog.append(rightclimbmotor.get());
    outputLog.append(rightclimbmotor.get());

    climberSwitch.setBoolean(ClimberLimitSwitch.get());
  }

  public void climbretractspeed(double power) {
    rightclimbmotor.set(-power);
    leftclimbmotor.set(-power);
  }

  public boolean hasClimbed() {
    return ClimberLimitSwitch.get();
  }

//   public boolean hasRetracted() {
//     return 
//   }
}


