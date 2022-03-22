/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FiringSolution.FiringSolution;
import frc.robot.FiringSolution.FiringSolutionManager;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class CalcSpeedAndAngle extends CommandBase {
  /**
   * Creates a new Enable_Shooter_Power.
   */

  private Shooter shooter;
  private Hood hood;
  private Limelight lim;
  private FiringSolutionManager manager;


  public CalcSpeedAndAngle(Shooter shooterSubsystem, Hood hoodSubsystem, Limelight limelight) {
    shooter = shooterSubsystem;
    hood = hoodSubsystem;
    lim = limelight;
    addRequirements(shooter, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    FiringSolution firingSolution = manager.calcNewSolution(lim.getDistance());
    shooter.setSpeed(firingSolution.getflywheelSpeed());
    hood.setTargetPosition(firingSolution.gethoodPosition());
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
    return (shooter.isAbleToEject() && hood.isAtTarget());
  }
}