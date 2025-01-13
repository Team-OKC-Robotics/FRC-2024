package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax rightclimbmotor;
    private final SparkMax leftclimbmotor;
    private final DigitalInput LeftClimberLimitSwitch;
    private final DigitalInput RightClimberLimitSwitch;
    
    public double target_climb;

    private final RelativeEncoder leftclimbencoder;
    private final RelativeEncoder rightclimbencoder;

    // private ShuffleboardTab tab = Shuffleboard.getTab("climber");
    // private GenericEntry climberSwitch = tab.add("climber switch", false).getEntry();
    


public ClimberSubsystem() {
    rightclimbmotor = new SparkMax(Constants.ClimberConstants.rightclimbermotorID, MotorType.kBrushless);
    leftclimbmotor = new SparkMax(Constants.ClimberConstants.leftclimbermotorID, MotorType.kBrushless);

    SparkMaxConfig rightmotorconfig = new SparkMaxConfig();
    SparkMaxConfig leftmotorconfig = new SparkMaxConfig();

    leftmotorconfig.inverted(false).idleMode(IdleMode.kBrake);
    rightmotorconfig.inverted(true).idleMode(IdleMode.kBrake);

    leftclimbmotor.configure(rightmotorconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightclimbmotor.configure(rightmotorconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    RightClimberLimitSwitch = new DigitalInput(3);
    LeftClimberLimitSwitch = new DigitalInput(2);

    leftclimbencoder = leftclimbmotor.getEncoder();
    rightclimbencoder = rightclimbmotor.getEncoder();

}

public void rightclimbspeed(double power) {
    rightclimbmotor.set(power);
    

}

public void leftclimbspeed(double power) {
    leftclimbmotor.set(power);
}

public void stopclimb() {
    rightclimbmotor.set(0);
    leftclimbmotor.set(0);
}


public double getSpeed() {
    return rightclimbmotor.get();

}

public double getPower() {
    return rightclimbmotor.get();
}

@Override
  public void periodic() {
    // This method will be called once per scheduler run
    

    
  }

  public void ClimbIt(double speed) {
    rightclimbmotor.set(speed);
    leftclimbmotor.set(speed);
  }
  
  public void resetleftencoder() {
    leftclimbencoder.setPosition(0);
    
}

  public void resetrightencoder() {
    rightclimbencoder.setPosition(0);
  }


  public boolean hasleftHit() {
    return !LeftClimberLimitSwitch.get();
  }
 
  public boolean hasrightHit() {
    return !RightClimberLimitSwitch.get();
  }

  public boolean isleftClose() {
    return leftclimbencoder.getPosition() < 25;
  }

  public boolean isrightClose() {
    return rightclimbencoder.getPosition() < 25;
  }


}



