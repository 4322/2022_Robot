package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DrivePolar extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new Drive_Manual.
   *
   * @param subsystem The subsystem used by this command.
   */

  private final Drive drive;
  private final double driveDeg;
  private final double speed;
  private final double rotationDeg;
  private final double driveSec;

  private Timer timer = new Timer();

  public DrivePolar(Drive drivesubsystem, double driveDeg, double speed, 
                    double rotationDeg, double driveSec) {
    drive = drivesubsystem;
    this.driveDeg = driveDeg;
    this.speed = speed;
    this.rotationDeg = rotationDeg;
    this.driveSec = driveSec;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Drive.getDriveMode() != Drive.DriveMode.fieldCentric) {

      // This command is only used in auto, so it's OK to not restore the 
      // previous drive mode.
      drive.setDriveMode(Drive.DriveMode.fieldCentric);
    }
    timer.reset();
    timer.start();
    drive.resetRotatePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double driveRad = Math.toRadians(driveDeg); 
    double x = speed * Math.cos(driveRad);
    double y = speed * Math.sin(driveRad);
    double headingErrorDeg = Drive.boundDegrees(rotationDeg - drive.getAngle());
    drive.driveAutoRotate(x, y, headingErrorDeg, DriveConstants.autoRotateToleranceDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(driveSec);
  }
}
