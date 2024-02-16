package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    private final CANSparkMax intakemotor;
    private final DigitalInput limitSwitchBeamBrake;

public IntakeSubsystem() {
    intakemotor = new CANSparkMax(Constants.IntakeConstants.intakemotorID, CANSparkLowLevel.MotorType.kBrushless);
    intakemotor.restoreFactoryDefaults();
    intakemotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    intakemotor.setInverted(true);
    limitSwitchBeamBrake = new DigitalInput(Constants.IntakeConstants.limitSwitchBeanBrakeChannel);
}

public void setSpeed(double power) {
    intakemotor.set(power);
}

public void stopIntake() {
    intakemotor.set(0);
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

public boolean getBeamBrakeState(){
    return limitSwitchBeamBrake.get();
}


public Command runIntake(double Speed){
    return run(() -> {
        setSpeed(Speed);
    });
}
@Override
public void periodic() {
    
    }

public void SetIntake(double speed) {
    intakemotor.set(speed);
} 
}
