package frc.robot.commands;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ResetFieldCentric extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Drive driveSubsystem;
  private final double offset;
  private final boolean runWhenEnabled;

  public ResetFieldCentric(Drive driveSubsystem, double offset, boolean runWhenEnabled) {
    this.driveSubsystem = driveSubsystem;
    this.offset = offset;
    this.runWhenEnabled = runWhenEnabled;
    // no need to interrupt other commands when changing drive mode
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.isDisabled() || runWhenEnabled) {
      driveSubsystem.resetFieldCentric(offset);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean runsWhenDisabled() {
    return true;  // allow field orientation to be set before start of match
  }
}
