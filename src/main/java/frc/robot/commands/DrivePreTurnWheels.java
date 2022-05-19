package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DrivePreTurnWheels extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new Drive_Manual.
   *
   * @param subsystem The subsystem used by this command.
   */

  private final Drive drive;
  private final double driveDeg;

  private Timer timer = new Timer();

  public DrivePreTurnWheels(Drive drivesubsystem, double driveDeg) {
    drive = drivesubsystem;
    this.driveDeg = driveDeg;
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
    double x = Constants.DriveConstants.smallNonZeroSpeed * Math.cos(driveRad);
    double y = Constants.DriveConstants.smallNonZeroSpeed * Math.sin(driveRad);
    drive.driveAutoRotate(x, y, 0, DriveConstants.autoRotateToleranceDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(Constants.DriveConstants.wheelPreRotateSec);
  }
}
