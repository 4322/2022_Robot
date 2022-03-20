package frc.robot.commands;

import frc.robot.subsystems.Driveunbun;
import frc.robot.subsystems.SwerveDrive.SwerveHelper;
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
  private final double offset;
  private final double time; // seconds

  private Timer timer = new Timer();

  public DriveRobotCentric(Driveunbun drivesubsystem, double m_x, double m_y, double m_rotate, double m_timeSeconds, double offsetDeg) {
    driveunbun = drivesubsystem;
    x = m_x;
    y = m_y;
    rotate = m_rotate;
    offset = offsetDeg;
    time = m_timeSeconds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveunbun);
  }

  public DriveRobotCentric(Driveunbun drivesubsystem, double m_x, double m_y, double m_rotate, double m_timeSeconds) {
    this(drivesubsystem, m_x, m_y, m_rotate, m_timeSeconds, 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveHelper.setToBotCentric(offset);
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
    SwerveHelper.setToFieldCentric();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);
  }
}
