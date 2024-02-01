package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax climbmotor;
    
    private double climbSetpoint = 0;
    private boolean climbStopped = true;
    

    public RelativeEncoder climbEncoder;
    
    public boolean isEnabled;


public ClimberSubsystem() {
    climbmotor = new CANSparkMax(11, MotorType.kBrushless);
    climbEncoder = climbmotor.getEncoder();
    climbEncoder.setPosition(0);
    climbmotor.setInverted(true);

}

public void setEncoderPosition() {
    climbEncoder.setPosition(0);
}

public void setspeed(double speed) {
    climbmotor.set(speed);

}

public double getPosition(){
    return climbEncoder.getPosition();
}

public void climbStopped() {
        climbmotor.set(0);

    }
}



