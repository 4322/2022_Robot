package frc.robot.commands;

import frc.robot.subsystems.Driveunbun;
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

  private final Driveunbun driveunbun;
  private final double x;
  private final double y;
  private final double rotate;
  private final double seconds;
  private Driveunbun.DriveMode previousDriveMode;
  private Driveunbun.DriveMode driveMode;

  private Timer timer = new Timer();

  public DriveRobotCentric(Driveunbun drivesubsystem, double x, double y, double rotate, 
          double seconds, Driveunbun.DriveMode driveMode) {
    driveunbun = drivesubsystem;
    this.x = x;
    this.y = y;
    this.rotate = rotate;
    this.driveMode = driveMode;
    this.seconds = seconds;
    addRequirements(driveunbun);
  }

  public DriveRobotCentric(Driveunbun drivesubsystem, double x, double y, double rotate, double seconds) {
    this(drivesubsystem, x, y, rotate, seconds, Driveunbun.DriveMode.frontCamCentric);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousDriveMode = driveunbun.getDriveMode();
    driveunbun.setDriveMode(driveMode);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveunbun.drive(x, y, rotate);
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
