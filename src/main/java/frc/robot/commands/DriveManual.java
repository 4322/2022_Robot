package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Driveunbun;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive.SwerveHelper;
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
  private final Limelight limelight;

  public DriveManual(Driveunbun drivesubsystem, Limelight limelightsubsystem) {
    driveunbun = drivesubsystem;
    limelight = limelightsubsystem;

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
      //
      
      // cache hardware status for consistency in logic
      final double driveRawX = driveX = RobotContainer.driveStick.getX();
      final double driveRawY = driveY = RobotContainer.driveStick.getY();
      final double rotateRawX = RobotContainer.rotateStick.getX();
      final double rotateRawY = RobotContainer.rotateStick.getY();
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
      } else if (driveunbun.isDrivingWithSideCams()) {
        // move slowly in side cam driving mode for percise cargo alignment
        driveX *= DriveConstants.sideCamDriveScaleFactor;
        driveY *= DriveConstants.sideCamDriveScaleFactor;
        rotate *= DriveConstants.sideCamRotationScaleFactor;
      } else {
        rotate *= DriveConstants.normalRotationScaleFactor;        
      }

      // determine drive mode
      // Kill, Limelight, Polar, Normal
      if ((Driveunbun.getDriveMode() == Driveunbun.DriveMode.killFieldCentric) ||
          (Driveunbun.getDriveMode() == Driveunbun.DriveMode.sideKillFieldCentric)) {
          rotate =  90 - Math.toDegrees(Math.atan2(-driveRawY, driveRawX));
          if (Driveunbun.getDriveMode() == Driveunbun.DriveMode.sideKillFieldCentric) {
            rotate += 90;
          }
          double error = SwerveHelper.boundDegrees(rotate - driveunbun.getAngle());
          if (Math.abs(error) > 90) {
            //Drive other way to minimize rotation
            error = SwerveHelper.boundDegrees(error + 180);
          }
          if (driveRawR < DriveConstants.drivePolarDeadband) {
            driveunbun.stop();
            driveunbun.resetRotatePID();
          } else {
            driveunbun.driveAutoRotate(-driveX, driveY, error, DriveConstants.manualRotateToleranceDegrees);
          }
      }
      else if ((Driveunbun.getDriveMode() == Driveunbun.DriveMode.limelightFieldCentric) &&
                limelight.getTargetVisible()) {
        double error = limelight.getHorizontalDegToTarget();
        driveunbun.driveAutoRotate(-driveX, driveY, error, DriveConstants.limeRotNotMovingToleranceDegrees);
      }
      else if (rotateRawR >= DriveConstants.rotatePolarDeadband) {
        // Get angle of joystick as desired rotation target
        rotate =  90 - Math.toDegrees(Math.atan2(-rotateRawY, rotateRawX));
        double error = SwerveHelper.boundDegrees(rotate - driveunbun.getAngle());
        driveunbun.driveAutoRotate(-driveX, driveY, error, DriveConstants.manualRotateToleranceDegrees);
      } else {
        // normal drive
        driveunbun.resetRotatePID();
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
