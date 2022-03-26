package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class StartFiringOverride extends CommandBase {
  private Conveyor conveyor;

  public StartFiringOverride(Conveyor conveyorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    conveyor = conveyorSubsystem;

    // stop updating firing solution so everything can stabilize
    addRequirements(conveyor);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.enableConveyor();
  }

  @Override
  public void execute() {
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.stop();
  }

  // Run until time has expired or interrupted
  @Override
  public boolean isFinished() {
    return false;
  }
}
