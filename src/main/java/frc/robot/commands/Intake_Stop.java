package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class Intake_Stop extends InstantCommand {
  /**
   * Creates a new Collector_Stop.
   */

   private Intake intake;

  public Intake_Stop(Intake intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = intakeSubsystem;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stop(); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
