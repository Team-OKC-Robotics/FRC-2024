package frc.robot.subsystems.superstructure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.subsystems.pivot.*;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.BooleanSupplier;

public class SuperstructureToState extends SequentialCommandGroup {
    private final Superstructure m_superstructure;

    private final SuperState m_targetState;

    private BooleanSupplier m_climberWait = () -> true;
    
    private BooleanSupplier m_intakeWait = () -> true;
    private BooleanSupplier m_shooterWait = () -> true;
    
    private BooleanSupplier m_swerveWait = () -> true;
    
    private BooleanSupplier m_pivotUntil = () -> false;
    private BooleanSupplier m_pivotWait = () -> true;
    private BooleanSupplier m_intakeUntil = () -> false;
    private BooleanSupplier m_shooterUntil = () -> false;
    
    private BooleanSupplier m_swerveUntil = () -> false;

    public SuperstructureToState(Superstructure superstructure,SuperState targetState){
        m_superstructure = superstructure;
        m_targetState = targetState;

        //ClimberSubsystem climber = superstructure.m_climber;
        //FeederSubsystem feeder = superstructure.m_feeder;
        IntakeSubsystem intake = superstructure.m_intake;

        //LEDSubsystem LED = superstructure.m_LED;
        ShooterSubsystem shooter = superstructure.m_shooter;
        //ElevatorSubsystem elevator = superstructure.m_elevator;
        SwerveSubsystem swerve = superstructure.m_swerve;

        PivotSubsystem pivot = superstructure.m_pivot;


        Command initCmd = Commands.runOnce(() -> m_superstructure.updateState(m_targetState));

        determineConditions();

        Command shooterCmd = Commands.waitUntil(m_shooterWait).andThen(superstructure.m_shooter.shootIt(m_targetState.shooter.speed).until(m_shooterUntil));
        //Command feederCmd = Commands.waitUntil(m_feederWait).andThen(superstructure.m_feeder.runFeeder(m_targetState.feed.power).until(m_feederUntil));
        //Command elevatorCmd = Commands.waitUntil(m_elevatorWait).andThen(superstructure.m_elevator.setAngle(m_targetState.elevator.angle).until(m_elevatorUntil));
        Command intakeCmd = Commands.waitUntil(m_intakeWait).andThen(superstructure.m_intake.runIntake(m_targetState.intake.power).until(m_intakeUntil));
        // Command pivotCmd = Commands.waitUntil(m_pivotWait).andThen(superstructure.m_pivot.setAngle(m_targetState.pivot.angle).until(m_pivotUntil));
        //Command climberCmd = Commands.waitUntil(m_climberWait).andThen(superstructure.m_climber.setHeight(m_targetState.climb.height)).until(m_climberUntil);


         ParallelCommandGroup commandGroup = new ParallelCommandGroup(
                                                                      intakeCmd, 
                                                                      shooterCmd
                                                                      
                                                              
                                                                     );

        // addCommands(initCmd, commandGroup.withTimeout(30));
    }

    private void determineConditions() {
    
        IntakeSubsystem intake = m_superstructure.m_intake;
        ShooterSubsystem shooter = m_superstructure.m_shooter;
        
        if(m_targetState == SuperState.INTAKE_NOTE) {
            m_intakeWait = () -> true;
            
           // m_intakeUntil = IntakeLimitSwitch.get();
        }
    } 
}
    // if(m_targetState == SuperState.SHOOT_AMP) {
    //        m_feederWait = () -> (elevator.getAngle() >= m_targetState.elevator.angle && shooter.getSpeed() >= m_targetState.shoot.speed);
    //    }
//
    //    if(m_targetState == SuperState.SHOOT_SPEAKER) {
    //        m_feederWait = () -> (elevator.getAngle() >= m_targetState.elevator.angle && shooter.getSpeed() >= m_targetState.shoot.speed);
    //    }
//
    //    if(m_targetState == SuperState.SHOOT_PROTECTED) {
    //        m_feederWait = () -> (elevator.getAngle() >= m_targetState.elevator.angle && shooter.getSpeed() >= m_targetState.shoot.speed);
    //    }
//
    //    if (m_targetState == SuperState.SOURCE_INTAKE) {
    //        m_shooterWait = () -> (elevator.getAngle() >= (m_targetState.elevator.angle));
    //        m_shooterUntil = feeder::getBeamBrakeState;
    //        m_feederWait = () -> true;
    //        m_feederUntil = feeder::getBeamBrakeState;
    //    }
//
    //    if (m_targetState == SuperState.GROUND_INTAKE) {
    //        m_intakeWait = () -> true;
    //        m_feederWait = () -> (intake.getIntakePistonPosition() == IntakeSubsystem.intakePistonDownPosition);
    //        m_feederUntil = feeder::getBeamBrakeState;
    //    }
//
    //    if (m_targetState == SuperState.CLIMB_REACH) {
    //        m_climberWait = () -> (elevator.getAngle() >= (m_targetState.elevator.angle) && (intake.getIntakePistonPosition() == IntakeSubsystem.intakePistonUpPosition));
    //    }
//
    //}

//}