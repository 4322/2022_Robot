// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.naming.spi.InitialContextFactoryBuilder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private PowerDistribution pdp = new PowerDistribution();
  private ShuffleboardTab tab;
  private NetworkTableEntry ampsTab;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    if (Constants.ultrasonicEnabled) {
      Ultrasonic.setAutomaticMode(true);
    }

    m_robotContainer = new RobotContainer();

    if (Constants.debug) {
      InitCurrentDisplay();
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
    if (Constants.debug) {
      UpdateCurrentDisplay();
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.disableSubsystems();
  }

  @Override
  public void disabledPeriodic() {}

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

    m_robotContainer.enableSubsystems();
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

  private void InitCurrentDisplay() {
    tab = Shuffleboard.getTab("PDP");

    ampsTab = tab.add("Current", 0)
    .withPosition(0,0)   
    .withSize(1,3)
    .getEntry();
  }

  private void UpdateCurrentDisplay() {
    String[] text = {"Right Front Drive: " + pdp.getCurrent(Constants.PDP.driveFrontRight),
                     "Left Front Drive:  " + pdp.getCurrent(Constants.PDP.driveFrontLeft),
                     "Left Rear Drive: " + pdp.getCurrent(Constants.PDP.driveRearLeft),
                     "Right Rear Drive: " + pdp.getCurrent(Constants.PDP.driveRearRight),
                     "Right Front Rotation: " + pdp.getCurrent(Constants.PDP.rotationFrontRight),
                     "Left Front Rotation:  " + pdp.getCurrent(Constants.PDP.rotationFrontLeft),
                     "Left Rear Rotation: " + pdp.getCurrent(Constants.PDP.rotationRearLeft),
                     "Right Rear Rotation: " + pdp.getCurrent(Constants.PDP.rotationRearRight)};
    ampsTab.setStringArray(text);
  }
}
