/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.XboxController;

public class Shooter_ManualEject extends CommandBase {
  /**
   * Creates a new Enable_Shooter_Power.
   */

  private Shooter shooter;
  private double m_rpm;
  private XboxController m_copilot;


  public Shooter_ManualEject(Shooter shooterSubsystem, double rpm, XboxController copilot) {
    shooter = shooterSubsystem;
    m_rpm = rpm;
    m_copilot = copilot;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSpeed(m_rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isAbleToEject() && m_copilot.rt.get()) {
      shooter.enableKicker();
    } else {
      shooter.disableKicker();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    shooter.disableKicker();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;     
  }
}