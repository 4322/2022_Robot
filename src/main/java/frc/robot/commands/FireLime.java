package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.FiringSolution.FiringSolution;
import frc.robot.FiringSolution.FiringSolutionManager;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Driveunbun;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// This command must be scheduled to run as non-interruptable!

public class FireLime extends CommandBase {
    
  private Kicker kicker;
  private Conveyor conveyor;
  private Shooter shooter;
  private Hood hood;
  private Driveunbun driveunbun;
  private Limelight limelight;

  public FireLime(Kicker kickerSubsystem, Conveyor conveyorSubsystem, 
              Shooter shooterSubsystem, Hood hoodSubsystem, 
              Driveunbun driveunbun, Limelight limelight) {
    kicker = kickerSubsystem;
    conveyor = conveyorSubsystem;
    shooter = shooterSubsystem;
    hood = hoodSubsystem;
    this.driveunbun = driveunbun;
    this.limelight = limelight;

    // stop updating firing solution so everything can stabilize
    addRequirements(kicker, conveyor, shooter, hood);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (!limelight.getTargetVisible()) {
      conveyor.autoStop();
      kicker.stop();
      shooter.stop();
      hood.stop();
      return;
    }

    if (conveyor.isLoaded()) {
      FiringSolution firingSolution = FiringSolutionManager.getSingleton()
        .calcNewSolution(limelight.getDistance());
      kicker.setSpeed(firingSolution.getKickerSpeed());
      shooter.setSpeed(firingSolution.getFlywheelSpeed());
      hood.setTargetPosition(firingSolution.getHoodPosition());
      
      if (shooter.isAtSpeed() && kicker.isAtSpeed() && hood.isAtTarget() &&
          (Math.abs(limelight.getHorizontalDegToTarget()) <= 
           (driveunbun.isRobotMoving()? DriveConstants.limeRotMovingToleranceDegrees
                                      : DriveConstants.limeRotNotMovingToleranceDegrees))) {
        conveyor.shoot();
      } else {
        conveyor.autoStop();
      }
    } else {
      conveyor.autoAdvanceCargo();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.autoStop();
  }

  // Run until time has expired or interrupted
  @Override
  public boolean isFinished() {
    return false;  
  }
}
