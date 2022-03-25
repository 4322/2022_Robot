package frc.robot.commands;

import frc.robot.subsystems.Driveunbun;
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
  private final double angle;
  private final double speed;
  private final double rotate;
  private final double seconds;
  private Driveunbun.DriveMode previousDriveMode;

  private Timer timer = new Timer();

  public DrivePolar(Driveunbun drivesubsystem, double angle, double speed, 
                    double rotationAngle, double seconds) {
    driveunbun = drivesubsystem;
    this.angle = angle;
    this.speed = speed;
    rotate = rotationAngle;
    this.seconds = seconds;
    addRequirements(driveunbun);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousDriveMode = Driveunbun.getDriveMode();
    driveunbun.setDriveMode(Driveunbun.DriveMode.fieldCentric);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveunbun.drivePolar(angle, speed, rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveunbun.stop();
    driveunbun.setDriveMode(previousDriveMode);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(seconds);
  }
}
