package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Driveunbun;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Driveunbun.DriveMode;

public class SetDriveMode extends InstantCommand {
  /**
   * Creates a new Disable_Shooter.
   */

  private Shooter shooter;
  private Hood hood;
  private Kicker kicker;
  private Conveyor conveyor;
  private Driveunbun driveunbun;
  private Limelight limelight;
  private DriveMode newDriveMode;
  private static FireLime fireLime;

  public SetDriveMode(Kicker kickerSubsystem, Conveyor conveyorSubsystem,
      Shooter shooterSubsystem, Hood hoodSubsystem,
      Driveunbun driveunbun, Limelight limelight, DriveMode newDriveMode) {
    conveyor = conveyorSubsystem;
    shooter = shooterSubsystem;
    hood = hoodSubsystem;
    kicker = kickerSubsystem;
    this.driveunbun = driveunbun;
    this.limelight = limelight;
    this.newDriveMode = newDriveMode;
    addRequirements(conveyor, shooter, kicker, hood);
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
        boolean interruptable = false;
        driveunbun.setDriveMode(newDriveMode);
        if (fireLime == null) {
          fireLime = new FireLime(kicker, conveyor, shooter, hood, limelight);
          CommandScheduler.getInstance().schedule(interruptable, fireLime);
        }
        return;
      }
    }
    if (fireLime != null) {
      fireLime.cancel();
      fireLime = null;
    }
    conveyor.autoStop();
    kicker.stop();
    shooter.stop();
    hood.stop();
    driveunbun.setDriveMode(newDriveMode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
