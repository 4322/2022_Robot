/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FiringSolution.FiringSolution;
import frc.robot.FiringSolution.FiringSolutionManager;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class CalcFiringSolution extends InstantCommand {

  private Kicker kicker;
  private Shooter shooter;
  private Hood hood;
  private Limelight lim;
  private FiringSolutionManager manager = FiringSolutionManager.getSingleton();

  public CalcFiringSolution(Kicker kickerSubsystem, Shooter shooterSubsystem, 
                            Hood hoodSubsystem, Limelight limelight) {
    kicker = kickerSubsystem;
    shooter = shooterSubsystem;
    hood = hoodSubsystem;
    lim = limelight;
    addRequirements(kicker, shooter, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (lim.getTargetVisible()) {
      FiringSolution firingSolution = manager.calcNewSolution(lim.getDistance());
      kicker.setSpeed(firingSolution.getKickerSpeed());
      shooter.setSpeed(firingSolution.getFlywheelSpeed());
      hood.setTargetPosition(firingSolution.getHoodPosition());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}