package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveManual extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new Drive_Manual.
   *
   * @param subsystem The subsystem used by this command.
   */

  private final Drive drive;
  private final Limelight limelight;

  public DriveManual(Drive drivesubsystem, Limelight limelightsubsystem) {
    drive = drivesubsystem;
    limelight = limelightsubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
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
      //
      
      // cache hardware status for consistency in logic
      // X axis is horizontal for joysticks, but represents the front of the robot
      // (forward/reverse) for WPILib's calculations, so the values for Y and 
      // X are substituted to correct for this (90 degree rotation)
      final double driveRawX = driveX = RobotContainer.driveStick.getY();
      final double driveRawY = driveY = RobotContainer.driveStick.getX();
      final double rotateRawX = RobotContainer.rotateStick.getY();
      final double rotateRawY = RobotContainer.rotateStick.getX();
      final double rotateRawZ = RobotContainer.rotateStick.getZ();

      // calculate distance from center of joysticks
      final double driveRawR = Math.sqrt(driveRawX * driveRawX + driveRawY * driveRawY);
      final double rotateRawR = Math.sqrt(rotateRawX * rotateRawX + rotateRawY * rotateRawY);

      /* 
        cube drive joystick inputs to increase sensitivity
        x = smaller value
        y = greater value
        x = (y^3 / y) * x 
      */
      if (Math.abs(driveRawX) >= Math.abs(driveRawY)) {
        driveX = driveRawX * driveRawX * driveRawX;
        driveY = driveRawX * driveRawX * driveRawY;
      } else {
        driveX = driveRawY * driveRawY * driveRawX;
        driveY = driveRawY * driveRawY * driveRawY;
      }

      // Check for drive deadband.
      // Can't renormalize x and y independently because we wouldn't be able to drive diagonally
      // at low speed.
      if (driveRawR < DriveConstants.drivePolarDeadband) {
        driveX = 0;
        driveY = 0;
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

      if (Constants.demo.inDemoMode) {
        rotate *= Constants.demo.rotationScaleFactor;
        if (Constants.demo.driveMode == Constants.demo.DriveMode.SLOW_DRIVE) {
          driveX *= Constants.demo.driveScaleFactor;
          driveY *= Constants.demo.driveScaleFactor;
        } else {
          driveX = 0;
          driveY = 0;
        }
      } else if (drive.isDrivingWithSideCams()) {
        // move slowly in side cam driving mode for percise cargo alignment
        driveX *= DriveConstants.sideCamDriveScaleFactor;
        driveY *= DriveConstants.sideCamDriveScaleFactor;
        rotate *= DriveConstants.sideCamRotationScaleFactor;
      } else {
        rotate *= DriveConstants.normalRotationScaleFactor;        
      }

      // determine drive mode
      // Kill, Limelight, Polar, Normal
      if ((Drive.getDriveMode() == Drive.DriveMode.killFieldCentric) ||
          (Drive.getDriveMode() == Drive.DriveMode.sideKillFieldCentric)) {
          rotate =  90 - Math.toDegrees(Math.atan2(-driveRawY, driveRawX));
          if (Drive.getDriveMode() == Drive.DriveMode.sideKillFieldCentric) {
            rotate += 90;
          }
          double error = Drive.boundDegrees(rotate - drive.getAngle());
          if (Math.abs(error) > 90) {
            //Drive other way to minimize rotation
            error = Drive.boundDegrees(error + 180);
          }
          if (driveRawR < DriveConstants.drivePolarDeadband) {
            drive.stop();
            drive.resetRotatePID();
          } else {
            drive.driveAutoRotate(-driveX, driveY, error, DriveConstants.manualRotateToleranceDegrees);
          }
      }
      else if ((Drive.getDriveMode() == Drive.DriveMode.limelightFieldCentric) &&
                limelight.getTargetVisible()) {
        double error = limelight.getHorizontalDegToTarget();
        drive.driveAutoRotate(-driveX, driveY, error, DriveConstants.limeRotNotMovingToleranceDegrees);
      }
      else if (rotateRawR >= DriveConstants.rotatePolarDeadband) {
        // Get angle of joystick as desired rotation target
        rotate =  90 - Math.toDegrees(Math.atan2(-rotateRawY, rotateRawX));
        double error = Drive.boundDegrees(rotate - drive.getAngle());
        drive.driveAutoRotate(-driveX, driveY, error, DriveConstants.manualRotateToleranceDegrees);
      } else {
        // normal drive
        drive.resetRotatePID();
        drive.drive(-driveX, driveY, -rotate);
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
