/*

Vertical Position
Forward rotation (toward front) until hit bar
Reposition robot to line up hooks

*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;

public class ClimberEngage extends CommandBase {
  private final Climber climber;
  private final Drive drive;
  private boolean abort;

  public ClimberEngage(Climber climbsubsystem, Drive drivesubsystem) {
    climber = climbsubsystem;
    drive = drivesubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    abort = !climber.moveToPosition(Constants.ClimberConstants.firstEngage, Climber.climbMode.unloaded);

    // TODO: make sure that coordinates between unbun and wpi match
    drive.drive(0, 0.15, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return abort;
  }

}