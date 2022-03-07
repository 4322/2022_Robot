/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Conveyor;

public class Conveyor_Stop extends InstantCommand {
  /**
   * Creates a new Disable_Shooter.
   */

  private Conveyor conveyor;

  public Conveyor_Stop(Conveyor conveyorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    conveyor = conveyorSubsystem;
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      conveyor.disableConveyor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}