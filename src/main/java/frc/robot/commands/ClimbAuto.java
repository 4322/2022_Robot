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
    forwardSecondBar,
    backwardLatchSecond,
    forwardThirdBar,
    backwardLatchThird,
    forwardVertical,
    done,
    abort;
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
          if (climber.moveToPosition(ClimberConstants.forwardSecondBar, Climber.climbMode.loaded)) {
            currentMode = climberMode.forwardSecondBar;
          }
          else {
            currentMode = climberMode.abort;
          }
          break;
        case forwardSecondBar:
          if (climber.isAtTarget()) {
            if (climber.moveToPosition(ClimberConstants.backwardLatchSecond, Climber.climbMode.loaded)) {
              currentMode = climberMode.backwardLatchSecond;
            }
            else {
              currentMode = climberMode.abort;
            }
          }
          break;
        case backwardLatchSecond:
          if (climber.isAtTarget()) {
            if (climber.moveToPosition(ClimberConstants.forwardThirdBar, Climber.climbMode.loaded)) {
              currentMode = climberMode.forwardThirdBar;
            }
            else {
              currentMode = climberMode.abort;
            }
          }
          break;
        case forwardThirdBar:
          if (climber.isAtTarget()) {
            if (climber.moveToPosition(ClimberConstants.backwardLatchThird, Climber.climbMode.loaded)) {
              currentMode = climberMode.backwardLatchThird;
            }
            else {
              currentMode = climberMode.abort;
            }
          }
          break;
        case backwardLatchThird:
          if (climber.isAtTarget()) {
            if (climber.moveToPosition(ClimberConstants.forwardVertical, Climber.climbMode.loaded)) {
              currentMode = climberMode.forwardVertical;
            }
            else {
              currentMode = climberMode.abort;
            }
          }
          break;
        case forwardVertical:
          if (climber.isAtTarget()) {
            climber.stop();
            currentMode = climberMode.done;
          }
          break;
        case done:
        case abort:
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
    climber.isClimbLocked() || (currentMode == climberMode.abort));
  }

}