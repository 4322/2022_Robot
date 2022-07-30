/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;

public class ClimberReset extends InstantCommand {
  /**
   * Creates a new Disable_Shooter.
   */

   private Climber climber;

  public ClimberReset(Climber climberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = climberSubsystem;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   climber.lockClimb();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }
}