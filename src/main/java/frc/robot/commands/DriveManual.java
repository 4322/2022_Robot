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
  private double rawX;
  private double rawY;
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
      rawX = RobotContainer.driveStick.getX();
      rawY = RobotContainer.driveStick.getY();

      // get distance from center of joystick
      polarDrive = Math.sqrt(rawX*rawX + rawY*rawY);

      /* 
        cube joystick inputs to increase sensitivity
        x = smaller value
        y = greater value
        x = (y^3 / y) * x 
      */
      if (Math.abs(rawX) >= Math.abs(rawY)) {
        driveX = -rawX*rawX*rawX; // reverse polarity of drive x axis
        driveY = rawX*rawX*rawY;
      } else {
        driveX = -rawY*rawY*rawX;
        driveY = rawY*rawY*rawY;
      }

      if (Constants.driveTwoJoystick) {
        // Uses pythagorean theorem to get deadband in any direction
        rotTo = Math.sqrt(Math.pow(RobotContainer.rotateStick.getX(), 2) + 
          Math.pow(RobotContainer.rotateStick.getY(), 2)) >= rotDeadband;
        rotate = RobotContainer.rotateStick.getZ();
      } else {
        rotate = RobotContainer.driveStick.getZ();
      }

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
        rotate =  90 - Math.toDegrees(Math.atan2(-RobotContainer.rotateStick.getY(), RobotContainer.rotateStick.getX()));

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
