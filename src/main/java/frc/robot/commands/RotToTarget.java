/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants.DriveConstants;

public class RotToTarget extends CommandBase {
  /**
   * Creates a new Enable_Shooter_Power.
   */

  private Drive drive;
  private Limelight lim;
  private Timer timeout = new Timer();
  private double timeoutLength;


  public RotToTarget(Drive driveSubsystem, Limelight limelight, double timeoutSeconds) {
    drive = driveSubsystem;
    lim = limelight;
    timeoutLength = timeoutSeconds;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeout.reset();
    timeout.start();
    drive.resetRotatePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lim.getTargetVisible()) {
      double error = lim.getHorizontalDegToTarget();
      drive.driveAutoRotate(0, 0, error, DriveConstants.limeRotNotMovingToleranceDegrees);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !lim.getTargetVisible() ||
           (Math.abs(lim.getHorizontalDegToTarget()) <= DriveConstants.limeRotNotMovingToleranceDegrees) ||
           timeout.hasElapsed(timeoutLength);
  }
}