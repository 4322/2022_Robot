package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Hood;

public class HoodReset extends CommandBase {
  /**
   * Creates a new Hood_Manual.
   */

  private Hood hood;
  private boolean firstReset = false;
  private boolean secondResetStarted = false;
  private boolean secondReset = false;
  private boolean targetSet = false;
  private Timer timer = new Timer();

  public HoodReset(Hood hoodSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    hood = hoodSubsystem;
    addRequirements(hood);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    hood.clearInitialHome();
    hood.moveHome();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!firstReset && !hood.isAtHome()) {
      hood.setHoodPower(HoodConstants.homingPower);
    } else if (!firstReset && hood.isAtHome()) {
      firstReset = true;
      hood.setCurrentPosition(0);
    } else if (!secondResetStarted && hood.isAtHome() && !targetSet) {
      hood.setTargetPosition(1000, true);
      targetSet = true;
    } else if (!secondResetStarted && hood.isAtTarget() && targetSet) {
      hood.setHoodPower(HoodConstants.secondHomingPower);
      secondResetStarted = true;
    } else if (secondResetStarted && hood.isAtHome()) {
      secondReset = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (secondReset) {
      hood.stop();
      hood.setInitiallyHomed();
      return true;
    }
    else if (timer.hasElapsed(Constants.HoodConstants.homingTimeout)) {
      hood.stop();
      return true;
    }
    return false;
  }
}