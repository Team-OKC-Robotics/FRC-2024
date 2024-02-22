package frc.robot.subsystems.superstructure;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
//import frc.robot.subsystems.Feeder.FeederSubsystem.FeederState;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;
//import frc.robot.subsystems.Climber.ClimberSubsystem.ClimberState;


public enum SuperState {

    /*
    States of Subsystems
    Intake - RETRACTED/EXTENDED
    Elevator - MAXANGLE/MIDANGLE/MINANGLE
    Feeder - FORWARD, OFF, REVERSE
    Shooter - HIGHPOWER, MIDPOWER, LOWPOWER, REVERSEDINTAKE, OFF
    Climber - RETRACTED/EXTENDED
     */
    INTAKE_NOTE(0,
        IntakeState.FORWARD, ShooterState.OFF),
    //GROUND_INTAKE(1,
    //        IntakeState.EXTENDED, FeederState.FORWARD, ElevatorState.MINANGLE, ShooterState.OFF, ClimberState.RETRACTED),
    //SOURCE_INTAKE(2,
    //        IntakeState.RETRACTED, FeederState.OFF, ElevatorState.MAXANGLE, ShooterState.REVERSEDINTAKE, ClimberState.RETRACTED),
    SCORE_AMP_SETUP(3,
            IntakeState.OFF,  ShooterState.LOWPOWER),
    SCORE_SPEAKER_SETUP(4,
            IntakeState.OFF, ShooterState.MIDPOWER),
    //SCORE_STAGE_PROTECTED_SETUP (5,
    //        IntakeState.RETRACTED, FeederState.OFF, ElevatorState.MIDANGLE, ShooterState.HIGHPOWER, ClimberState.RETRACTED),
    //CLIMB_REACH(6,
    //        IntakeState.RETRACTED, FeederState.OFF, ElevatorState.MINANGLE, ShooterState.OFF, ClimberState.EXTENDED),
    SHOOT_AMP(7,
          IntakeState.OFF, ShooterState.LOWPOWER),
//
    SHOOT_SPEAKER(8,
          IntakeState.OFF, ShooterState.MIDPOWER);
//
    //SHOOT_PROTECTED(9,
    //      IntakeState.RETRACTED, FeederState.FORWARD, ElevatorState.MIDANGLE, ShooterState.HIGHPOWER, ClimberState.RETRACTED);
//
    
public final int idx;
public final IntakeState intake;
public final ShooterState shooter;
    

    private SuperState(int idx, IntakeState intake, ShooterState shooter){
        this.idx = idx;
        this.intake = intake;
        this.shooter = shooter;
        
    }
}
