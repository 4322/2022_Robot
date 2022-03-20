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
  private Timer currentPosTimer = new Timer();

  private enum resetStates {
    firstDown,
    goingUp,
    settingTarget,
    secondDown,
    secondAtHome
  }
  private resetStates currentState = resetStates.firstDown;

  public HoodReset(Hood hoodSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    hood = hoodSubsystem;
    addRequirements(hood);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    overrideTimer.reset();
    overrideTimer.start();
    currentPosTimer.reset();
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
          currentPosTimer.start();
          currentState = resetStates.settingTarget;
        }
        break;
      case settingTarget:
        if (currentPosTimer.hasElapsed(0.025)) {
          hood.setTargetPosition(1000, true);
          currentPosTimer.stop();
          currentState = resetStates.goingUp;
        }
        break;
      case goingUp:
        if (hood.isAtTarget()) {
          hood.setHoodPower(HoodConstants.secondHomingPower);
          currentState = resetStates.secondDown;
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
      hood.stop();
      return true;
    }
    return false;
  }
}