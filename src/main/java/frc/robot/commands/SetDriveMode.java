package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Driveunbun;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
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
  private DriveMode newDriveMode;

  public SetDriveMode(Kicker kickerSubsystem, Conveyor conveyorSubsystem,
      Shooter shooterSubsystem, Hood hoodSubsystem,
      Driveunbun driveunbun, DriveMode newDriveMode) {
    conveyor = conveyorSubsystem;
    shooter = shooterSubsystem;
    hood = hoodSubsystem;
    kicker = kickerSubsystem;
    this.driveunbun = driveunbun;
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
      }
      if (newDriveMode != DriveMode.limelightFieldCentric) {
        conveyor.autoStop();
        kicker.stop();
        shooter.stop();
        hood.stop();
      }
      driveunbun.setDriveMode(newDriveMode);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
