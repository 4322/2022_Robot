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
  private Timer timer = new Timer();
  private enum resetStates {
    firstDown,
    firstAtHome,
    secondUp,
    secondAtTarget,
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
    timer.reset();
    timer.start();
    hood.clearInitialHome();
    hood.moveHome();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (!firstReset && !hood.isAtHome()) {
    //   hood.setHoodPower(HoodConstants.homingPower);
    // } else if (!firstReset && hood.isAtHome()) {
    //   firstReset = true;
    //   hood.setCurrentPosition(0);
    // } else if (!secondResetStarted && hood.isAtHome() && !targetSet) {
    //   hood.setTargetPosition(1000, true);
    //   targetSet = true;
    // } else if (!secondResetStarted && hood.isAtTarget() && targetSet) {
    //   hood.setHoodPower(HoodConstants.secondHomingPower);
    //   secondResetStarted = true;
    // } else if (secondResetStarted && hood.isAtHome()) {
    //   secondReset = true;
    // }

    switch(currentState) {
      case firstDown:
        hood.setHoodPower(HoodConstants.homingPower);
        if (hood.isAtHome()) {
          currentState = resetStates.firstAtHome;
        }
        break;
      case firstAtHome:
        hood.setCurrentPosition(0);
        currentState = resetStates.secondUp;
        break;
      case secondUp:
        hood.setTargetPosition(500);
        if (hood.isAtTarget()) {
          currentState = resetStates.secondAtTarget;
        }
        break;
      case secondAtTarget:
        hood.setHoodPower(HoodConstants.secondHomingPower);
        currentState = resetStates.secondDown;
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
    else if (timer.hasElapsed(Constants.HoodConstants.homingTimeout)) {
      hood.stop();
      return true;
    }
    return false;
  }
}