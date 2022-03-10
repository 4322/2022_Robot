package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Kicker;

public class KickerStop extends InstantCommand {
    
  private Kicker kicker;
  private Conveyor conveyor;

  public KickerStop(Kicker kickerSubsystem, Conveyor conveyorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    kicker = kickerSubsystem;
    conveyor = conveyorSubsystem;
    addRequirements(kicker, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kicker.stop();
    conveyor.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }
}
