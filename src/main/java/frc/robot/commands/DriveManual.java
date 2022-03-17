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
  private double driveRawX;
  private double driveRawY;
  private double rotationRawX;
  private double rotationRawY;
  private double rotationRawZ;
  private boolean autoRotateDriveStick;
  private double driveX;
  private double driveY;
  private double polarDrive;
  private boolean rotTo = false;
  private double rotate;

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
    if (Constants.joysticksEnabled) {
      // top left button
      autoRotateDriveStick = RobotContainer.rotateStick.getRawButtonPressed(5);

      driveRawX = RobotContainer.driveStick.getX();
      driveRawY = RobotContainer.driveStick.getY();
      rotationRawX = RobotContainer.rotateStick.getX();
      rotationRawY = RobotContainer.rotateStick.getY();
      rotationRawZ = RobotContainer.rotateStick.getZ();

      // get distance from center of joystick
      polarDrive = Math.sqrt(driveRawX*driveRawX + driveRawY*driveRawY);

      /* 
        cube joystick inputs to increase sensitivity
        x = smaller value
        y = greater value
        x = (y^3 / y) * x 
      */
      if (Math.abs(driveRawX) >= Math.abs(driveRawY)) {
        driveX = -driveRawX*driveRawX*driveRawX; // reverse polarity of drive x axis
        driveY = driveRawX*driveRawX*driveRawY;
      } else {
        driveX = -driveRawY*driveRawY*driveRawX;
        driveY = driveRawY*driveRawY*driveRawY;
      }

      if (!autoRotateDriveStick) {
        // Uses pythagorean theorem to get deadband in any direction
        rotTo = Math.sqrt(Math.pow(rotationRawX, 2) + 
          Math.pow(rotationRawY, 2)) >= rotDeadband;
      } else {
        rotTo = true;
      }

      rotate = rotationRawZ;

      if (
          (Math.abs(polarDrive) < DriveConstants.polarManualDeadband) &&
          (Math.abs(rotate) < twistDeadband) &&
          (!rotTo)
          ) {
        driveunbun.stop();
        return;
      }


      if (!rotTo) {
        if (Math.abs(rotate) < twistDeadband) {
            rotate = 0;
        }
        else if (rotate > 0) {
            rotate = (rotate - twistDeadband) / (1 - twistDeadband);  // rescale to full positive range
        }
        else {
            rotate = (rotate + twistDeadband) / (1 - twistDeadband);  // rescale to full negative range
        } 

        driveunbun.drive(driveX, driveY, 
          -rotate*rotate*rotate); // reverse polarity of rotation on joystick

      } else {

        // Get angle of joystick
        if (autoRotateDriveStick) {
          rotate =  90 - Math.toDegrees(Math.atan2(-driveRawY, driveRawX));
        } else {
          rotate =  90 - Math.toDegrees(Math.atan2(-rotationRawY, rotationRawX));
        }

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
