package frc.robot.subsystems.superstructure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.*;
import frc.robot.subsystems.swervedrive.*;

public class Superstructure {

   
    public final IntakeSubsystem m_intake;
    
    public final SwerveSubsystem m_swerve;

    public final ShooterSubsystem m_shooter;

    public final PivotSubsystem m_pivot;
    

    public SuperState m_prevState = SuperState.INTAKE_NOTE;
//
    private SuperState m_curState = SuperState.INTAKE_NOTE;




    public Superstructure( IntakeSubsystem intake,
                            ShooterSubsystem shooter,
                            PivotSubsystem pivot,
                          SwerveSubsystem swerve) {
        
        m_intake = intake;
        
        m_swerve = swerve;
        
        m_shooter = shooter;

        m_pivot = pivot;
        //m_feeder.setDefaultCommand(m_feeder.runFeeder(FeederSubsystem.FeederState.OFF.power));
       m_intake.setDefaultCommand(m_intake.runIntake(IntakeSubsystem.IntakeState.OFF.power));
       m_shooter.setDefaultCommand(m_shooter.shootIt(ShooterSubsystem.ShooterState.OFF.speed));
       m_pivot.setDefaultCommand(m_pivot.setAngle(PivotSubsystem.PivotState.MINANGLE.angle));
       // m_elevator.setDefaultCommand(m_elevator.setAngle(ElevatorSubsystem.ElevatorState.MINANGLE.angle));
    }

    public Command toState(SuperState state){
        return new SuperstructureToState(this, state);
    }

    protected void updateState(SuperState newState) {
        System.out.println(
                "[SS] updateState - WAS " + m_prevState +
                        ", FROM " + m_curState +
                        " TO " + newState);
        m_prevState = m_curState;
        m_curState = newState;
    }

    public SuperState getPrevState() {
        return m_prevState;
    }

    public SuperState getCurState() {
        return m_curState;
    }

   

}



