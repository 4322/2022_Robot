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
      final double driveRawX = RobotContainer.driveStick.getX();
      final double driveRawY = RobotContainer.driveStick.getY();
      final double rotateRawX = RobotContainer.rotateStick.getX();
      final double rotateRawY = RobotContainer.rotateStick.getY();
      final double rotateRawZ = RobotContainer.rotateStick.getZ();

      // get distance from center of joysticks
      final double driveRawR = Math.sqrt(driveRawX * driveRawX + driveRawY * driveRawY);
      final double rotateRawR = Math.sqrt(rotateRawX * rotateRawX + rotateRawY * rotateRawY);

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

      if (driveunbun.getDrivingWithSideCams()) {
        driveX *= DriveConstants.camLimiter;
        driveY *= DriveConstants.camLimiter;
      }

      if ((rotateRawR >= DriveConstants.rotatePolarDeadband) && !driveunbun.getDrivingWithSideCams()) {
        // Get angle of joystick as desired rotation target
        rotate =  90 - Math.toDegrees(Math.atan2(-rotateRawY, rotateRawX));
        driveunbun.driveAutoRotate(driveX, driveY, rotate);
        return;
      }

      final double twistDeadband = DriveConstants.twistDeadband;
      if ((driveRawR < DriveConstants.drivePolarDeadband) &&
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
