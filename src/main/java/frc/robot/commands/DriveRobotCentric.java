package frc.robot.commands;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveRobotCentric extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new Drive_Manual.
   *
   * @param subsystem The subsystem used by this command.
   */

  private final Drive drive;
  private final double x;
  private final double y;
  private final double rotate;
  private final double seconds;
  private Drive.DriveMode previousDriveMode;
  private Drive.DriveMode driveMode;

  private Timer timer = new Timer();

  public DriveRobotCentric(Drive drivesubsystem, double x, double y, double rotate, 
          double seconds, Drive.DriveMode driveMode) {
    drive = drivesubsystem;
    this.x = x;
    this.y = y;
    this.rotate = rotate;
    this.driveMode = driveMode;
    this.seconds = seconds;
    addRequirements(drive);
  }

  public DriveRobotCentric(Drive drivesubsystem, double x, double y, double rotate, double seconds) {
    this(drivesubsystem, x, y, rotate, seconds, Drive.DriveMode.frontCamCentric);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousDriveMode = Drive.getDriveMode();
    drive.setDriveMode(driveMode);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.drive(x, y, rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    drive.setDriveMode(previousDriveMode);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(seconds);
  }
}
