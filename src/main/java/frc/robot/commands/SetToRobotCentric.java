package frc.robot.commands;

import frc.robot.subsystems.Driveunbun;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetToRobotCentric extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Driveunbun driveSubsystem;
  private final double offset;

  public SetToRobotCentric(Driveunbun driveSubsystem, double offsetDeg) {
    this.driveSubsystem = driveSubsystem;
    offset = offsetDeg;
    // no need to interrupt other commands when changing drive mode
  }

  public SetToRobotCentric(Driveunbun driveSubsystem) {
    this(driveSubsystem, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.setToRobotCentric(offset);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}
