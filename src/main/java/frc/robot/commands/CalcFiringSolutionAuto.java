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
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight.LedMode;

public class CalcFiringSolutionAuto extends CommandBase {
  /**
   * Creates a new Enable_Shooter_Power.
   */

  private Kicker kicker;
  private Shooter shooter;
  private Hood hood;
  private Limelight lim;
  private FiringSolutionManager manager = FiringSolutionManager.getSingleton();
  private boolean targetSet = false;


  public CalcFiringSolutionAuto(Kicker kickerSubsystem, Shooter shooterSubsystem, 
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
    lim.setLed(LedMode.On);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lim.getTargetVisible()) {
      FiringSolution firingSolution = manager.calcNewSolution(lim.getDistance());
      kicker.setSpeed(firingSolution.getKickerSpeed());
      shooter.setSpeed(firingSolution.getFlywheelSpeed());
      hood.setTargetPosition(firingSolution.getHoodPosition());
      targetSet = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return targetSet;
  }
}