package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Hood;

public class HoodSet extends CommandBase {
  /**
   * Creates a new Hood_Manual.
   */

  private final Hood hood;
  private final double target;

  public HoodSet(Hood hoodSubsystem, double m_target) {
    // Use addRequirements() here to declare subsystem dependencies.
    hood = hoodSubsystem;
    target = m_target;
    addRequirements(hood);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.setTargetPosition(target);
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
    return (Math.abs(hood.getPosition() - target) < HoodConstants.hoodTolerance);
  }
}