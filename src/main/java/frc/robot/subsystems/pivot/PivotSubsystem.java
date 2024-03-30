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
    private final CANSparkMax ampdevicemotor;
    private final PIDController AmpPidController;
    private final PIDController PivotPIDController;
    private final DutyCycleEncoder AmpEncoder;
    private final DutyCycleEncoder pivotencoder;

    private double targetAmpangle = 24;
    private double targetPivotangle = 57;

    private State currentState = State.AMP_IN;
    private State targetState= State.AMP_IN;
    

    private ShuffleboardTab pivottab = Shuffleboard.getTab("pivot");
    private ShuffleboardTab amptab = Shuffleboard.getTab("amp");
  
    private GenericEntry AmpDeviceEncoder = amptab.add("amp encoder", 0).getEntry();
  
    private GenericEntry pivotabsoluteencoder = pivottab.add("absolute encoder", 0).getEntry();

    private GenericEntry CurrentState = amptab.add("Current State", "Amp In").getEntry();

public PivotSubsystem() {
   
    pivotmotor = new CANSparkMax(Constants.PivotConstants.pivotmotorID, CANSparkLowLevel.MotorType.kBrushless);
    ampdevicemotor = new CANSparkMax(Constants.AmpConstants.ampdevicemotorID, CANSparkLowLevel.MotorType.kBrushless);

    pivotmotor.restoreFactoryDefaults();
    ampdevicemotor.restoreFactoryDefaults();
   
    pivotmotor.setInverted(false);
    ampdevicemotor.setInverted(true);
    
    pivotencoder = new DutyCycleEncoder(9);
    AmpEncoder = new DutyCycleEncoder(0);
  
    pivotmotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    ampdevicemotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    
   
    PivotPIDController = new PIDController(0.04, 0.0001, 0);
    AmpPidController = new PIDController(0.04, 0.0001, 0);
    
    

    pivotmotor.set(0);
    ampdevicemotor.set(0);
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
    return pivotencoder.getAbsolutePosition()*360;
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
    AmpDeviceEncoder.setDouble(getDevicePosition());

    CurrentState.setString(currentState.name());

    PivotIttoAngle(targetPivotangle);
    PivotAmpToAngle(targetAmpangle);

    if (currentState == State.AMP_IN && targetState == State.AMP_ENGAGED) {
        currentState = State.AMP_IN_PIVOT_MOVING;
    }

    if(currentState == State.AMP_IN_PIVOT_MOVING && targetState == State.AMP_ENGAGED) {  //if the amp is in and the pivot is moving and the target state is amp engaged
        targetPivotangle = 25;  //move pivot to 25
    if(getPivotAngle() < 27 ) {  //once pivot is less than 27 degrees 
        currentState = State.AMP_MOVING_PIVOT_OUT;  //the state is now amp moving pivot out
        }
    }

    if(currentState == State.AMP_MOVING_PIVOT_OUT && targetState == State.AMP_ENGAGED) {
        targetAmpangle = -25;
    if(getDevicePosition() < -23) {
        currentState = State.AMP_OUT_PIVOT_MOVING;
    }
    }

    if(currentState == State.AMP_OUT_PIVOT_MOVING && targetState == State.AMP_ENGAGED) {
        targetPivotangle = 53;
    if(getPivotAngle() > 52) {
        currentState = State.AMP_ENGAGED;
    }
    }



    if (currentState == State.AMP_ENGAGED && targetState == State.AMP_IN) {
        currentState = State.AMP_OUT_PIVOT_MOVING;
    }

    if(currentState == State.AMP_OUT_PIVOT_MOVING && targetState == State.AMP_IN) {  //if the amp is in and the pivot is moving and the target state is amp engaged
        targetPivotangle = 25;  //move pivot to 30
    if(getPivotAngle() < 27 ) {  //once pivot is less than 35 degrees 
        currentState = State.AMP_MOVING_PIVOT_OUT;  //the state is now amp moving pivot out
        }
    }

    if(currentState == State.AMP_MOVING_PIVOT_OUT && targetState == State.AMP_IN) {
        targetAmpangle = 24;
    if(getDevicePosition() > 22) {
        currentState = State.AMP_IN_PIVOT_MOVING;
    }
    }

    if(currentState == State.AMP_IN_PIVOT_MOVING && targetState == State.AMP_IN) {
        targetPivotangle = 57;
    if(getPivotAngle() > 55) {
        currentState = State.AMP_IN;
    }
    }

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

public void SetTargetPivotAngle(double angle) {
    if(currentState == State.AMP_IN) {
        targetPivotangle = angle;
}
}

public double getPivotAngle() {
    double rawvalue = pivotencoder.getAbsolutePosition();
    if (rawvalue > 0.5) { //bc the absolute encoder is messed up :(
        rawvalue = rawvalue -1;
    } 
    return -rawvalue *338 + 33.1; //some weird ahh math, I know
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
     
    if(currentState == State.AMP_IN) {
        return true;
    } else {
        return false;
    }
}

public double getDevicePosition() {
    double rawvalue = AmpEncoder.getAbsolutePosition();
    if (rawvalue > 0.5) { //bc the absolute encoder is messed up :(
        rawvalue = rawvalue -1;
    } 
    return -rawvalue *100; //some weird ahh math, I know
}
}
