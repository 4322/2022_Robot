package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.Constants;

public class Hood_Manual extends CommandBase {
  /**
   * Creates a new Hood_Manual.
   */

   private Hood shooterHood;
   private double power;

  public Hood_Manual(Hood hoodSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    shooterHood = hoodSubsystem;
    addRequirements(shooterHood);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    power = RobotContainer.coPilot.leftStick.getY();
    if (Math.abs(power) < Constants.HoodConstants.manualDeadband) {
      power = 0;
    }
    shooterHood.setHoodPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}