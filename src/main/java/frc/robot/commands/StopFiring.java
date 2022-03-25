/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class StopFiring extends InstantCommand {
  /**
   * Creates a new Disable_Shooter.
   */

   private Shooter shooter;
   private Hood hood;
   private Kicker kicker;
   private Conveyor conveyor;

  public StopFiring(Kicker kickerSubsystem, Conveyor conveyorSubsystem,
                    Shooter shooterSubsystem, Hood hoodSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    conveyor = conveyorSubsystem;
    shooter = shooterSubsystem;
    hood = hoodSubsystem;
    kicker = kickerSubsystem;
    addRequirements(conveyor, shooter, kicker, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.stop();
    kicker.stop();
    shooter.stop();
    hood.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}