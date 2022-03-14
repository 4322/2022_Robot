package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Driveunbun;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveManual extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new Drive_Manual.
   *
   * @param subsystem The subsystem used by this command.
   */

  private final Driveunbun driveunbun;
  private final double twistDeadband = DriveConstants.twistDeadband;
  private final double rotDeadband = DriveConstants.rotateToDeadband; // Deadband for turning to angle of joystick

  public DriveManual(Driveunbun drivesubsystem) {
    driveunbun = drivesubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveunbun);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double driveX;
    double driveY;
    double rotate;

    if (Constants.joysticksEnabled) {
      double driveRawX = RobotContainer.driveStick.getX();
      double driveRawY = RobotContainer.driveStick.getY();
      double rotateRawX = RobotContainer.rotateStick.getX();
      double rotateRawY = RobotContainer.rotateStick.getY();
      double rotateRawZ = RobotContainer.rotateStick.getZ();

      if (driveunbun.getDrivingWithSideCams()) {
        driveRawX = driveRawX * DriveConstants.camLimiter;
        driveRawY = driveRawY * DriveConstants.camLimiter;
      }

      // get distance from center of joysticks
      double driveRawR = Math.sqrt(driveRawX * driveRawX + driveRawY * driveRawY);
      double rotateRawR = Math.sqrt(rotateRawX * rotateRawX + rotateRawY * rotateRawY);

      /* 
        cube joystick inputs to increase sensitivity
        x = smaller value
        y = greater value
        x = (y^3 / y) * x 
      */
      if (Math.abs(driveRawX) >= Math.abs(driveRawY)) {
        driveX = -driveRawX * driveRawX * driveRawX; // reverse polarity of drive x axis
        driveY = driveRawX * driveRawX * driveRawY;
      } else {
        driveX = -driveRawY * driveRawY * driveRawX;
        driveY = driveRawY * driveRawY * driveRawY;
      }

      if ((rotateRawR >= rotDeadband) && !driveunbun.getDrivingWithSideCams()) {
        // Get angle of joystick as desired rotation target
        rotate =  90 - Math.toDegrees(Math.atan2(-rotateRawY, rotateRawX));
        driveunbun.driveAutoRotate(driveX, driveY, rotate);
        return;
      }

      if ((driveRawR < DriveConstants.polarManualDeadband) &&
          (Math.abs(rotateRawZ) < twistDeadband)) {
        driveunbun.stop();
        return;
      }

      if (Math.abs(rotateRawZ) < twistDeadband) {
          rotate = 0;
      }
      else if (rotateRawZ > 0) {
          rotate = (rotateRawZ - twistDeadband) / (1 - twistDeadband);  // rescale to full positive range
      }
      else {
          rotate = (rotateRawZ + twistDeadband) / (1 - twistDeadband);  // rescale to full negative range
      } 

      driveunbun.drive(driveX, driveY, -rotate * rotate * rotate);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
