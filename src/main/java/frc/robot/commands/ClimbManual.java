/*

Vertical Position
Forward rotation (toward front) until hit bar
Reposition robot to line up hooks

*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ClimbManual extends CommandBase {

  private final Climber climber;
  private double speed;

  public ClimbManual(Climber climbsubsystem) {
    climber = climbsubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!climber.isClimbLocked()) {
      speed = RobotContainer.coPilot.leftStick.getY();
      if (Math.abs(speed) < ClimberConstants.manualDeadband) {
        speed = 0;
      }
      // spread max climber power across full joystick range for increased sensitivity
      speed *= ClimberConstants.kMaxRange;

      climber.setClimberSpeed(speed);
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
    return false; 
  }

}