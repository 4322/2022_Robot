package frc.robot.commands;

import frc.robot.subsystems.Driveunbun;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ResetFieldCentric extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Driveunbun driveSubsystem;
  private final double offset;

  public ResetFieldCentric(Driveunbun driveSubsystem, double offset) {
    this.driveSubsystem = driveSubsystem;
    this.offset = offset;
    // no need to interrupt other commands when changing drive mode
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.resetFieldCentric(offset);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean runsWhenDisabled() {
    return true;  // allow field orientation to be set before start of match
  }
}
