package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class KickerAutoStart extends CommandBase {
    
  private Kicker kicker;
  private Conveyor conveyor;
  private Shooter shooter;
  private boolean finished = false;

  public KickerAutoStart(Kicker kickerSubsystem, Conveyor conveyorSubsystem, Shooter shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    kicker = kickerSubsystem;
    conveyor = conveyorSubsystem;
    shooter = shooterSubsystem;
    addRequirements(kicker, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    if (shooter.isAbleToEject()) {
      kicker.intake();
      conveyor.enableConveyor();
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // stop movement if autonomous command is interrupted
    kicker.stop();
    conveyor.stop();
  }

  // Run until interrupted
  @Override
  public boolean isFinished() {
    return finished;     
  }
}
