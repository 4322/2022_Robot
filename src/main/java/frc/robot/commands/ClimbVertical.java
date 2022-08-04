/*

Vertical Position
Forward rotation (toward front) until hit bar
Reposition robot to line up hooks

*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ClimbVertical extends CommandBase {
  private final Climber climber;
  private Timer overrideTimer = new Timer();

  public ClimbVertical(Climber climbsubsystem) {
    climber = climbsubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    overrideTimer.reset();
    overrideTimer.start();
    climber.moveToPosition(ClimberConstants.forwardFirstBar, 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
    climber.unlockClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((climber.isAtTarget()) || (overrideTimer.hasElapsed(ClimberConstants.overrideTime)));
  }

}