package frc.robot.commands;

import frc.robot.subsystems.Driveunbun;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetToRobotCentric extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Driveunbun driveSubsystem;

  public SetToRobotCentric(Driveunbun driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // no need to interrupt other commands when changing drive mode
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.setToRobotCentric();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}
