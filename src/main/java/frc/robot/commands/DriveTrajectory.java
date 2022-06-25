package frc.robot.commands;
  
import frc.robot.subsystems.Drive;

import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveTrajectory extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new Drive_Manual.
   *
   * @param subsystem The subsystem used by this command.
   */

  private final Drive drive;
  private final Trajectory driveTrajectory;
  private final boolean stop;

  private List<State> trajectoryStates;
  private int idx = 0;

  public DriveTrajectory(Drive drivesubsystem, Trajectory trajectory, boolean stopOnCompletion) {
    drive = drivesubsystem;
    driveTrajectory = trajectory;
    stop = stopOnCompletion;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trajectoryStates = driveTrajectory.getStates();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if ((idx < trajectoryStates.size()) || drive.isAtTarget()) {
      idx += 1;
    }

    drive.drive(trajectoryStates.get(idx));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (stop) {
      drive.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((idx == trajectoryStates.size()) || drive.isAtTarget());
  }
}
