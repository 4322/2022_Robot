package frc.robot.commands;

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
  private final double angle;
  private final double speed;
  private final double rotate;
  private final double time; // seconds

  private Timer timer = new Timer();

  public DrivePolar(Driveunbun drivesubsystem, double m_angle, double m_speed, double m_rotationAngle, double m_timeSeconds) {
    driveunbun = drivesubsystem;
    angle = m_angle;
    speed = m_speed;
    rotate = m_rotationAngle;
    time = m_timeSeconds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveunbun);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveHelper.setToFieldCentric();
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);
  }
}
