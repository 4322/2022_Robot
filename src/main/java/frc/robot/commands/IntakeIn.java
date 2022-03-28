package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class IntakeIn extends CommandBase {
    
    private Intake intake;
    private Conveyor conveyor;

    public IntakeIn(Intake intakeSubsystem, Conveyor conveyorSubsystem) {
    intake = intakeSubsystem;
    conveyor = conveyorSubsystem;

    // Don't require conveyor to continue shooting during manual intake.
    addRequirements(intake);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.manualAdvanceCargo();  // also starts intake
  }

  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.manualStop();  // also stops intake
  }

  // Run until interrupted
  @Override
  public boolean isFinished() {
    return false;
  }
}
