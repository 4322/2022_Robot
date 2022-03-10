/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class StopSpeedAndAngle extends InstantCommand {
  /**
   * Creates a new Disable_Shooter.
   */

   private Shooter shooter;
   private Hood hood;

  public StopSpeedAndAngle(Shooter shooterSubsystem, Hood hoodSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = shooterSubsystem;
    hood = hoodSubsystem;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stop();
    hood.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}