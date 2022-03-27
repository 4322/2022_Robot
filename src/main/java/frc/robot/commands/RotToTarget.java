/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Driveunbun;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.LedMode;
import frc.robot.Constants.DriveConstants;

public class RotToTarget extends CommandBase {
  /**
   * Creates a new Enable_Shooter_Power.
   */

  private Driveunbun driveunbun;
  private Limelight lim;


  public RotToTarget(Driveunbun driveSubsystem, Limelight limelight) {
    driveunbun = driveSubsystem;
    lim = limelight;
    addRequirements(driveunbun);
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
      double error = lim.getHorizontalDegToTarget();
      driveunbun.driveAutoRotate(0, 0, error);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(lim.getHorizontalDegToTarget()) < DriveConstants.limelightRotateToleranceDegrees;
  }
}