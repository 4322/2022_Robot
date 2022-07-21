package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ClimbAuto extends CommandBase {

  private climberMode currentMode;
  private final Climber climber;
  private Timer overrideTimer = new Timer();

  public enum climberMode {
    stopped,
    forward1,
    backward1,
    forward2,
    done;
  }

  public ClimbAuto(Climber climbsubsystem) {
    climber = climbsubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setCurrentPosition(0);
    currentMode = climberMode.stopped;
    overrideTimer.reset();
    overrideTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (currentMode) {
      case stopped:
        climber.moveToPosition(ClimberConstants.forward1);
        currentMode = climberMode.forward1;
        break;
      case forward1:
        if (climber.isAtTarget()) {
          climber.moveToPosition(ClimberConstants.backward1);
          currentMode = climberMode.backward1;
        }
        break;
      case backward1:
        if (climber.isAtTarget()) {
          climber.moveToPosition(ClimberConstants.forward2);
          currentMode = climberMode.forward2;
        }
        break;
      case forward2:
        if (climber.isAtTarget()) {
          climber.stop();
          currentMode = climberMode.done;
        }
        break;
      case done:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((currentMode == climberMode.done) || (overrideTimer.hasElapsed(ClimberConstants.overrideTime)));
  }

}