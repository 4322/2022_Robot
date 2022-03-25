/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FiringSolution.FiringSolution;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class SetFiringSolution extends CommandBase {
  /**
   * Creates a new Enable_Shooter_Power.
   */

  private Shooter shooter;
  private Hood hood;
  private Kicker kicker;
  private FiringSolution firingSolution;


  public SetFiringSolution(Kicker kickerSubsystem, Shooter shooterSubsystem, Hood hoodSubsystem, 
                          FiringSolution m_firingSolution) {
    shooter = shooterSubsystem;
    hood = hoodSubsystem;
    kicker = kickerSubsystem;
    firingSolution = m_firingSolution;
    addRequirements(kicker, shooter, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kicker.setSpeed(firingSolution.getKickerSpeed());
    shooter.setSpeed(firingSolution.getFlywheelSpeed());
    hood.setTargetPosition(firingSolution.getHoodPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (shooter.isAbleToEject() && kicker.isAbleToEject() && hood.isAtTarget());
  }
}