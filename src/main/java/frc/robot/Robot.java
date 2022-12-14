// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private ShuffleboardTab tab;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    tab = Shuffleboard.getTab("Enabled Subsystems");

    subsystemEnabled("Comp Mode", 0, 0, !Constants.debug);
    subsystemEnabled("Drivebase", 1, 0, Constants.driveEnabled);
    subsystemEnabled("Shooter", 2, 0, Constants.shooterEnabled);
    subsystemEnabled("Hood", 3, 0, Constants.hoodEnabled);
    subsystemEnabled("Intake", 4, 0, Constants.intakeEnabled);
    subsystemEnabled("Kicker", 5, 0, Constants.kickerEnabled);
    subsystemEnabled("Conveyor", 6, 0, Constants.conveyorEnabled);
    subsystemEnabled("Climber", 7, 0, Constants.climberEnabled);

    subsystemEnabled("Joysticks", 0, 1, Constants.joysticksEnabled);
    subsystemEnabled("Gyro", 1, 1, Constants.gyroEnabled);
    subsystemEnabled("Limelight", 2, 1, Constants.limelightEnabled);
    subsystemEnabled("Ball Sensor", 3, 1, Constants.ballSensorEnabled);

    m_robotContainer = new RobotContainer();

    DataLogManager.logNetworkTables(false);
  }

  // create new shuffleboard tab to show whether or not subsystem is enabled
  // also print error to driver station if not
  private void subsystemEnabled(String title, int posX, int posY, boolean enabled) {
    tab.add(title, enabled)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(posX, posY)
    .withSize(1, 1);

    if (!enabled) {
      DriverStation.reportError(title + " not enabled", false);
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.disableSubsystems();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.disabledPeriodic();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.enableSubsystems();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.hoodReset();
    m_robotContainer.enableSubsystems();
    m_robotContainer.climberReset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

}
