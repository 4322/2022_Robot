package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Driveunbun;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Driveunbun.DriveMode;

public class SetDriveMode extends InstantCommand {

  private Shooter shooter;
  private Driveunbun driveunbun;
  private DriveMode newDriveMode;
  private static FireLime fireLime;

  public SetDriveMode(Kicker kicker, Shooter shooter, Hood hood,
      Driveunbun driveunbun, Limelight limelight, DriveMode newDriveMode) {

    if (fireLime == null) {
      fireLime = new FireLime(kicker, shooter, hood, driveunbun, limelight);
    }
    this.shooter = shooter;
    this.driveunbun = driveunbun;
    this.newDriveMode = newDriveMode;
    // can't have any requirements or FireLime will be scheduled as interruptable
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveMode oldDriveMode = Driveunbun.getDriveMode();

    if (newDriveMode == DriveMode.limelightFieldCentric) {
      if ((oldDriveMode != DriveMode.limelightFieldCentric) && shooter.isRunning()) {
        // preset firing solution is running, deny limelight mode
        return;
      } else {
        driveunbun.setDriveMode(newDriveMode);
        if (!fireLime.isScheduled()) {
          boolean interruptable = false;
          fireLime.schedule(interruptable);
        }
        return;
      }
    }
    if (fireLime.isScheduled()) {
      fireLime.cancel();
    }
    driveunbun.setDriveMode(newDriveMode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
