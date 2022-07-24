package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Hood;

public class HoodReset extends CommandBase {
  /**
   * Creates a new Hood_Manual.
   */

  private Hood hood;
  private Timer overrideTimer = new Timer();
  private Timer statusTimer = new Timer();

  private enum resetStates {
    firstDown,
    goingUp,
    settingTarget,
    secondDown,
    secondAtHome
  }
  private resetStates currentState;

  public HoodReset(Hood hoodSubsystem) {
    hood = hoodSubsystem;
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = resetStates.firstDown;
    overrideTimer.reset();
    overrideTimer.start();
    hood.clearInitialHome();
    hood.moveHome();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(currentState) {
      case firstDown:
        if (hood.isAtHome()) {
          hood.setCurrentPosition(0);
          statusTimer.reset();  // clear accumulated time from prior hood reset
          statusTimer.start();
          currentState = resetStates.settingTarget;
        }
        break;
      case settingTarget:
        if (statusTimer.hasElapsed(Constants.statusLatencySec)) {
          hood.setTargetPosition(1000, true);
          currentState = resetStates.goingUp;
          statusTimer.reset();
        }
        break;
      case goingUp:
        if (statusTimer.hasElapsed(Constants.statusLatencySec)) {
          if (hood.isAtTarget()) {
            hood.moveHomeSlow();
            currentState = resetStates.secondDown;
          }
        }
        break;
      case secondDown:
        if (hood.isAtHome()) {
          currentState = resetStates.secondAtHome;
        }
        break;
      case secondAtHome:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentState == resetStates.secondAtHome) {
      hood.stop();
      hood.setInitiallyHomed();
      return true;
    }
    else if (overrideTimer.hasElapsed(Constants.HoodConstants.homingTimeout)) {
      DriverStation.reportError("Hood homing timed out", false);
      hood.stop();
      return true;
    }
    return false;
  }
}