/*

Vertical Position
Forward rotation (toward front) until hit bar
Reposition robot to line up hooks

Climb Auto
Continue clockwise until passing second bar
Counter clockwise to release previous hook and engage new hook
Clockwise until passing third bar
Counter clockwise to release previous hook and engage new hook
Clockwise until vertical to complete hang

*/

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
    floatingSecondBar,
    disengageFirstBar,
    floatingThirdBar,
    disengageSecondBar,
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
    currentMode = climberMode.stopped;
    overrideTimer.reset();
    overrideTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!climber.isClimbLocked()) {
      switch (currentMode) {
        case stopped:
          climber.moveToPosition(ClimberConstants.floatingSecondBar, Climber.climbMode.loaded);
          currentMode = climberMode.floatingSecondBar;
        break;
        case floatingSecondBar:
          if (climber.isAtTarget()) {
            climber.moveToPosition(ClimberConstants.disengageFirstBar, Climber.climbMode.loaded);
            currentMode = climberMode.disengageFirstBar;
          }
          break;
        case disengageFirstBar:
          if (climber.isAtTarget()) {
            climber.moveToPosition(ClimberConstants.floatingThirdBar, Climber.climbMode.loaded);
            currentMode = climberMode.floatingThirdBar;
          }
        break;
        case floatingThirdBar:
          if (climber.isAtTarget()) {
            climber.moveToPosition(ClimberConstants.disengageSecondBar, Climber.climbMode.loaded);
            currentMode = climberMode.disengageSecondBar;
          }
        break;
        case disengageSecondBar:
          if (climber.isAtTarget()) {
            currentMode = climberMode.done;
          }
        break;
        case done:
          break;
      }
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
    return ((currentMode == climberMode.done) || 
    (overrideTimer.hasElapsed(ClimberConstants.overrideTime)) || 
    climber.isClimbLocked());
  }

}