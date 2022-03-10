package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class IntakeIn extends CommandBase {
    
    private Intake intake;
    private Conveyor conveyor;

    public IntakeIn(Intake intakeSubsystem, Conveyor conveyorSubsystem) {
            // Use addRequirements() here to declare subsystem dependencies.
    intake = intakeSubsystem;
    conveyor = conveyorSubsystem;
    addRequirements(intake, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intake();
    if (!conveyor.getSensedBall()) {
      conveyor.enableConveyor();
    }
  }

  @Override
  public void execute() {
    if (conveyor.getSensedBall()) {
      conveyor.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    conveyor.stop();
  }

  // Run until interrupted
  @Override
  public boolean isFinished() {
    return false;     
  }
}
