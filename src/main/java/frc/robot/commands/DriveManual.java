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

    final double twistDeadband = DriveConstants.twistDeadband;
    final double rotDeadband = DriveConstants.rotateToDeadband; // Deadband for turning to angle of joystick
    double driveRawX;
    double driveRawY;
    double rotationRawX;
    double rotationRawY;
    double rotationRawZ;
    double driveScaledX;
    double driveScaledY;
    double driveX;
    double driveY;
    double polarDrive;
    boolean rotTo = false;
    double rotate;
  
    if (Constants.joysticksEnabled) {
      driveRawX = driveScaledX = RobotContainer.driveStick.getX();
      driveRawY = driveScaledY = RobotContainer.driveStick.getY();
      rotationRawX = RobotContainer.rotateStick.getX();
      rotationRawY = RobotContainer.rotateStick.getY();
      rotationRawZ = RobotContainer.rotateStick.getZ();

      if (driveunbun.getDrivingWithCams()) {
        driveScaledX *= DriveConstants.camLimiter;
        driveScaledY *= DriveConstants.camLimiter;
      }

      // get distance from center of joystick
      polarDrive = Math.sqrt(driveRawX*driveRawX + driveRawY*driveRawY);

      /* 
        cube joystick inputs to increase sensitivity
        x = smaller value
        y = greater value
        x = (y^3 / y) * x 
      */
      if (Math.abs(driveScaledX) >= Math.abs(driveScaledY)) {
        driveX = -driveScaledX*driveScaledX*driveScaledX; // reverse polarity of drive x axis
        driveY = driveScaledX*driveScaledX*driveScaledY;
      } else {
        driveX = -driveScaledY*driveScaledY*driveScaledX;
        driveY = driveScaledY*driveScaledY*driveScaledY;
      }

      // Uses pythagorean theorem to get deadband in any direction
      rotTo = Math.sqrt(Math.pow(rotationRawX, 2) + 
        Math.pow(rotationRawY, 2)) >= rotDeadband;

      if (
          (Math.abs(polarDrive) < DriveConstants.polarManualDeadband) &&
          (Math.abs(rotationRawZ) < twistDeadband) &&
          (!rotTo)
          ) {
        driveunbun.stop();
        return;
      }

      if (!rotTo) {
        if (Math.abs(rotationRawZ) < twistDeadband) {
            rotate = 0;
        }
        else if (rotationRawZ > 0) {
            rotate = (rotationRawZ - twistDeadband) / (1 - twistDeadband);  // rescale to full positive range
        }
        else {
            rotate = (rotationRawZ + twistDeadband) / (1 - twistDeadband);  // rescale to full negative range
        } 
        
        if (driveunbun.getDrivingWithCams()) {
          rotate *= DriveConstants.camLimiter;
        }

        driveunbun.drive(driveX, driveY, 
          -rotate*rotate*rotate); // reverse polarity of rotation on joystick

      } else {

        // Get angle of joystick
        rotate =  90 - Math.toDegrees(Math.atan2(-rotationRawY, rotationRawX));

        driveunbun.driveAutoRotate(driveX, driveY, 
          rotate);

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
