package frc.robot.subsystems.pivot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.subsystems.shooter.ShooterSubsystem.PIDF;

import java.util.function.DoubleSupplier;

public class PivotSubsystem extends SubsystemBase{
   // private final CANSparkMax leftpivot;
    private final CANSparkMax pivotmotor;

    private double targetAngle;

    private final PIDController PivotPIDController;
    private final RelativeEncoder pivotEncoder;
    private final DutyCycleEncoder pivotencoder;

    private ShuffleboardTab tab = Shuffleboard.getTab("pivot");
  //  private GenericEntry writeMode = tab.add("write mode", false).getEntry();

   // private final RelativeEncoder leftEncoder;
    private GenericEntry pivotabsoluteencoder = tab.add("absolute encoder", 0).getEntry();
public PivotSubsystem() {
   // leftpivot = new CANSparkMax(Constants.PivotConstants.leftpivotID, CANSparkLowLevel.MotorType.kBrushless);
    pivotmotor = new CANSparkMax(Constants.PivotConstants.pivotmotorID, CANSparkLowLevel.MotorType.kBrushless);
    pivotmotor.restoreFactoryDefaults();
   // rightpivot.restoreFactoryDefaults();
   // leftpivot.follow(rightpivot);
    pivotmotor.setInverted(false);
    
     pivotencoder = new DutyCycleEncoder(9);
  //  leftpivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
    pivotmotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    pivotEncoder = pivotmotor.getEncoder();
   // leftEncoder = leftpivot.getEncoder();
    PivotPIDController = new PIDController(0.03, 0.0001, 0);
    
    

    pivotmotor.set(0);
}



public void changeAngle(double power) {
    pivotmotor.set(power);
}

public void stopPivot() {
    pivotmotor.set(0);
}

public double getAngle() {
    return pivotEncoder.getPosition()*360;
}



// public Command setAngle(double degrees) {
//     targetAngle = degrees;
//     return run(() -> {
//         runPID(degrees);
//     });
// }

// public Command runManual(DoubleSupplier supplier) {
//     double power = supplier.getAsDouble();
//     return run(() -> {
//         changeAngle(power);
//     });

// }

public enum PivotState {
    MAXANGLE(80),
    MIDANGLE(50),
    MINANGLE(30);

    public double angle;

    private PivotState(double angle) {
        this.angle = angle;
    }
}

@Override
public void periodic() {
    pivotabsoluteencoder.setDouble(getPivotAngle());
}

public void PivotIt(double power) {
    //adds soft limits to avoid the pivot killing itself
    if (power > 0 && getPivotAngle() > 60) {
        pivotmotor.set(0);
        return; 
    }
    if (power < 0 && getPivotAngle() < 22) {
        pivotmotor.set(0);
        return;
        }

    pivotmotor.set(power);
    }

public void PivotIttoAngle(double angle) {
    PivotIt(PivotPIDController.calculate(getPivotAngle(), angle)); 
}


public double getPivotAngle() {
    double rawvalue = pivotencoder.getAbsolutePosition();
    if (rawvalue > 0.5) { //bc the absolute encoder is messed up :(
        rawvalue = rawvalue -1;
    } 
    return -rawvalue *338 + 33.1; //some weird ahh math, I know
}
}