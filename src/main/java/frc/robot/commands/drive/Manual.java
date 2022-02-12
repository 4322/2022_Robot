// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drivebase;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Manual extends CommandBase {

  private final Drivebase m_drivebase;

  public Manual(Drivebase drivebase) {
    m_drivebase = drivebase;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drivebase.drive(
      RobotContainer.pilotl.getX(), 
      RobotContainer.pilotl.getY(), 
      RobotContainer.pilotr.getDirectionDegrees(), 
      true
    );

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
