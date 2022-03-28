package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Driveunbun;
import frc.robot.subsystems.SwerveDrive.SwerveHelper;
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

  private final Driveunbun driveunbun;
  private final double driveDeg;
  private final double speed;
  private final double rotationDeg;
  private final double driveSec;

  private Timer timer = new Timer();

  public DrivePolar(Driveunbun drivesubsystem, double driveDeg, double speed, 
                    double rotationDeg, double driveSec) {
    driveunbun = drivesubsystem;
    this.driveDeg = driveDeg;
    this.speed = speed;
    this.rotationDeg = rotationDeg;
    this.driveSec = driveSec;
    addRequirements(driveunbun);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Driveunbun.getDriveMode() != Driveunbun.DriveMode.fieldCentric) {

      // This command is only used in auto, so it's OK to not restore the 
      // previous drive mode.
      driveunbun.setDriveMode(Driveunbun.DriveMode.fieldCentric);
    }
    timer.reset();
    timer.start();
    driveunbun.resetRotatePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // adjust for front of robot as 0 and clockwise rotation
    double driveRad = Math.toRadians(90 - driveDeg); 
    
    double x = speed * Math.cos(driveRad);
    double y = speed * Math.sin(driveRad);
    double error = SwerveHelper.boundDegrees(rotationDeg - driveunbun.getAngle());
    driveunbun.driveAutoRotate(-x, -y, error, DriveConstants.autoRotateToleranceDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveunbun.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(driveSec);
  }
}
