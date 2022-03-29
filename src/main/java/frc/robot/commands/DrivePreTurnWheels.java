package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Driveunbun;
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

  private final Driveunbun driveunbun;
  private final double driveDeg;

  private Timer timer = new Timer();

  public DrivePreTurnWheels(Driveunbun drivesubsystem, double driveDeg) {
    driveunbun = drivesubsystem;
    this.driveDeg = driveDeg;
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
    
    double x = Constants.DriveConstants.smallNonZeroSpeed * Math.cos(driveRad);
    double y = Constants.DriveConstants.smallNonZeroSpeed * Math.sin(driveRad);
    driveunbun.driveAutoRotate(-x, -y, 0, DriveConstants.autoRotateToleranceDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveunbun.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(Constants.DriveConstants.wheelPreRotateSec);
  }
}
