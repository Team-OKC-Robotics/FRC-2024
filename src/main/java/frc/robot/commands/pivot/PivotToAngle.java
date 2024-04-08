package frc.robot.commands.pivot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.*;

public class PivotToAngle extends Command{
    private final PivotSubsystem pivot;
    private double angle;

public PivotToAngle(PivotSubsystem pivot, double angle) {
        this.pivot = pivot;
        this.angle = angle;
        addRequirements(pivot);
    }

    @Override

    public void initialize() {
        pivot.SetTargetPivotAngle(angle);
    }
   
   @Override
   public void execute() {
       pivot.SetTargetPivotAngle(angle);
   }
   
   @Override
   public void end(boolean interuppted) {
       //pivot.PivotIt(0);
   }
   
   @Override
   public boolean isFinished() {
       return true;
    }
}
   
   
   
   
    



