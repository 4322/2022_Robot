package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class FirePreset extends CommandBase {
    
  private Kicker kicker;
  private Conveyor conveyor;
  private Shooter shooter;
  private Hood hood;
  private double fireSec;
  private double lastLogged = 0;
  private Timer firingTimer = new Timer();

  public FirePreset(Kicker kickerSubsystem, Conveyor conveyorSubsystem, 
              Shooter shooterSubsystem, Hood hoodSubsystem, double fireSec) {
    // Use addRequirements() here to declare subsystem dependencies.
    kicker = kickerSubsystem;
    conveyor = conveyorSubsystem;
    shooter = shooterSubsystem;
    hood = hoodSubsystem;
    this.fireSec = fireSec;

    // stop updating firing solution so everything can stabilize
    addRequirements(kicker, conveyor, shooter, hood);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    firingTimer.reset();
    firingTimer.start();
  }

  @Override
  public void execute() {
    if (conveyor.isLoaded()) {
      if (shooter.isAtSpeed() && kicker.isAtSpeed() && hood.isAtTarget()) {
        if (Math.abs(shooter.getSpeed() - lastLogged) > Constants.ShooterConstants.logInterval) {
          DataLogManager.log("Fired Shot:\n" +
                             "Time: " + DriverStation.getMatchTime() + "\n" +
                             "Shooter Speed: " + shooter.getSpeed() + "\n" +
                             "Kicker Speed: " + kicker.getSpeed() + "\n" +
                             "Hood Position: " + hood.getPosition());
          lastLogged = shooter.getSpeed();
        }
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
    if (fireSec > 0 && firingTimer.hasElapsed(fireSec)) {
      return true;
    } else {
      return false;  
    }   
  }
}
