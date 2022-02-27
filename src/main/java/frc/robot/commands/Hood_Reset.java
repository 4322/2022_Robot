package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class Hood_Reset extends CommandBase {
  /**
   * Creates a new Hood_Manual.
   */

  private Hood hood;
  private Timer timer = new Timer();

  public Hood_Reset(Hood hoodSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    hood = hoodSubsystem;
    addRequirements(hood);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    hood.setNotAtHome();
    hood.moveHome();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.setHoodPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (hood.isAtHome()) {
      hood.setAtHome();
      return true;
    }
    else if (timer.hasElapsed(Constants.HoodConstants.homingTimeout)) {
      return true;
    }
    return false;
  }
}