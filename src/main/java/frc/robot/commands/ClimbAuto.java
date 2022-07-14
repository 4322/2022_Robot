package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
    
    /** An example command that uses an example subsystem. */
public class ClimbAuto extends CommandBase {
    /**
     * Creates a new Climb_Manual.
     */
    
    private Climber climbAuto;
    private Timer overrideTimer = new Timer();
    private Timer currentPosTimer = new Timer();

    public enum climberMode {
        stopped,
        forward1,
        backward1,
        forward2,
        done;
    }
    private climberMode currentMode;
    

    
    private final Climber climber;

    public ClimbAuto(Climber climbsubsystem) {
        climber = climbsubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
}

// Called when the command is initially scheduled.
@Override
  public void initialize() {
    currentMode = climberMode.stopped;
    overrideTimer.reset();
    overrideTimer.start();
  }

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    switch(currentMode) {
      case stopped:
        if (climber.isAtTarget()) {
          currentPosTimer.reset();  // clear accumulated time from prior hood reset
          currentPosTimer.start();
          currentMode = climberMode.stopped;
        }
        break;
      case forward1:
        if (currentPosTimer.hasElapsed(0.025)) {
          currentMode = climberMode.forward1;
        }
        else currentMode = climberMode.done;
        break;
      case backward1:
        if (climber.isAtTarget()) {
          currentMode = climberMode.backward1;
        }
        else currentMode = climberMode.done;
        break;
      case forward2:
        if (climber.isAtTarget()) {
          currentMode = climberMode.forward2;
        }
        else currentMode = climberMode.done;
        break;
      case done:
        break;
    }
  }


}
