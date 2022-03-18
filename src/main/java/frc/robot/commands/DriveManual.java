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

      // cache hardware status for consistency in logic
      final double driveRawX = driveX = RobotContainer.driveStick.getX();
      final double driveRawY = driveY = RobotContainer.driveStick.getY();
      final double rotateRawX = RobotContainer.rotateStick.getX();
      final double rotateRawY = RobotContainer.rotateStick.getY();
      final double rotateRawZ = RobotContainer.rotateStick.getZ();

      // calculate distance from center of joysticks
      final double driveRawR = Math.sqrt(driveRawX * driveRawX + driveRawY * driveRawY);
      final double rotateRawR = Math.sqrt(rotateRawX * rotateRawX + rotateRawY * rotateRawY);

      // Check for drive deadband.
      // Can't renormalize x and y independently because we wouldn't be able to drive diagonally
      // at low speed.
      if (driveRawR < DriveConstants.drivePolarDeadband) {
        driveX = 0;
        driveY = 0;
      }

      /* 
        cube drive joystick inputs to increase sensitivity
        x = smaller value
        y = greater value
        x = (y^3 / y) * x 
      */
      if (Math.abs(driveRawX) >= Math.abs(driveRawY)) {
        driveX = driveX * driveX * driveX;
        driveY = driveX * driveX * driveY;
      } else {
        driveX = driveY * driveY * driveX;
        driveY = driveY * driveY * driveY;
      }

      // adjust for twist deadband
      final double twistDeadband = DriveConstants.twistDeadband;
      if (Math.abs(rotateRawZ) < twistDeadband) {
        rotate = 0;
      }
      else if (rotateRawZ > 0) {
        // rescale to full positive range
        rotate = (rotateRawZ - twistDeadband) / (1 - twistDeadband);
      }
      else {
        // rescale to full negative range
        rotate = (rotateRawZ + twistDeadband) / (1 - twistDeadband);
      }
      rotate = rotate * rotate * rotate;  // increase sensitivity

      // move slowly in side cam driving mode for percise cargo alignment
      if (driveunbun.getDrivingWithSideCams()) {
        driveX *= DriveConstants.sideCamDriveScaleFactor;
        driveY *= DriveConstants.sideCamDriveScaleFactor;
        rotate *= DriveConstants.sideCamRotationScaleFactor;
      } else {
        rotate *= DriveConstants.normalRotationScaleFactor;        
      }

      // determine drive mode
      if (rotateRawR >= DriveConstants.rotatePolarDeadband) {
        // Get angle of joystick as desired rotation target
        rotate =  90 - Math.toDegrees(Math.atan2(-rotateRawY, rotateRawX));
        driveunbun.driveAutoRotate(-driveX, driveY, rotate);
      } else if ((driveX == 0) && (driveY == 0) && (rotate == 0)) {
        // don't rotate wheels such that we trip over them when decelerating
        driveunbun.stop();
      } else {
        // normal drive
        driveunbun.drive(-driveX, driveY, -rotate);
      }
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
