// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.FiringSolution.FiringSolution;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.cameras.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Define controllers
  public static Joystick driveStick;
  public static Joystick rotateStick;
  public static XboxController coPilot = new XboxController(2);

  private static JoystickButton driveTopLeftButton;
  private static JoystickButton driveBottomLeftButton;
  private static JoystickButton driveTopRightButton;
  private static JoystickButton driveBottomRightButton;
  private static JoystickButton rotateButtonEleven;

  // The robot's subsystems and commands are defined here...
  private final Driveunbun driveunbun = new Driveunbun();
  private final Shooter shooter = new Shooter();
  private final Kicker kicker = new Kicker();
  private final Intake intake = new Intake();
  private final Hood hood = new Hood();
  private final Conveyor conveyor = new Conveyor();
  private final Webcams webcams = new Webcams();

  // Drive Commands
  private final DriveManual driveManual = new DriveManual(driveunbun);

  // Shooter Commands
  private final StopSpeedAndAngle stopSpeedAndAngle = new StopSpeedAndAngle(shooter, hood);

  // Intake Commands
  private final IntakeIn intakeIn = new IntakeIn(intake, conveyor);
  private final IntakeOut intakeOut = new IntakeOut(intake);

  // Hood Commands
  private final HoodReset hoodReset = new HoodReset(hood);

  // Kicker Commands
  private final KickerEnable kickerEnable = new KickerEnable(kicker, conveyor, shooter);

  // Firing Solutions
  // fender distances need to be remeasured
  // measurement from back of bumper for now
  private final FiringSolution fenderHigh = new FiringSolution(3000, 1800, 7.3);
  private final FiringSolution fenderLow = new FiringSolution(1200, 6000, 7.3);
  private final FiringSolution insideTarmac = new FiringSolution(3100, 4000, 84.75); // used to be 3100
  private final FiringSolution outsideTarmac = new FiringSolution(3500, 4000, 120);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Initialize subsystems after all of them have been constructed because the 
    // constructors lower CAN bus utilization to make configuration reliable.
    driveunbun.init();
    shooter.init();
    kicker.init();
    intake.init();
    hood.init();
    conveyor.init();

    // Configure the button bindings
    configureButtonBindings();

    if (Constants.driveEnabled) {
      driveunbun.setDefaultCommand(driveManual);
    }
  }

  public void disableSubsystems() {
    driveunbun.setCoastMode();
    hood.setCoastMode();
    conveyor.setCoastMode();
    intake.setCoastMode();
    kicker.setCoastMode();
  }

  public void enableSubsystems() {
    driveunbun.setBrakeMode();
    hood.setBrakeMode();
    conveyor.setBrakeMode();
    intake.setBrakeMode();
    kicker.setBrakeMode();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    coPilot.x.whenPressed(new SetSpeedAndAngle(shooter, hood, fenderHigh));
    coPilot.y.whenPressed(new SetSpeedAndAngle(shooter, hood, outsideTarmac));
    coPilot.b.whenPressed(new SetSpeedAndAngle(shooter, hood, insideTarmac));
    coPilot.lb.whenPressed(new SetSpeedAndAngle(shooter, hood, fenderLow));
    coPilot.a.whenPressed(stopSpeedAndAngle);

    coPilot.rb.whileHeld(intakeIn);
    coPilot.rt.whileHeld(intakeOut);

    coPilot.lt.whileHeld(kickerEnable);

    coPilot.back.whenPressed(hoodReset);

    if (Constants.joysticksEnabled) {
      driveStick = new Joystick(0);
      rotateStick = new Joystick(1);
      driveTopLeftButton = new JoystickButton(driveStick, 5);
      driveBottomLeftButton = new JoystickButton(driveStick, 3);
      driveBottomRightButton = new JoystickButton(driveStick, 4);
      driveTopRightButton = new JoystickButton(driveStick, 6);
      rotateButtonEleven = new JoystickButton(rotateStick, 11);
      driveTopLeftButton.whenPressed(new SetToFieldCentric(driveunbun));
      driveBottomLeftButton.whenPressed(new SetToRobotCentric(driveunbun, 90));
      driveBottomRightButton.whenPressed(new SetToRobotCentric(driveunbun, -90));
      driveTopLeftButton.whenPressed(webcams::resetCameras);
      driveTopRightButton.whenPressed(webcams::resetCameras);
      driveBottomLeftButton.whenPressed(webcams::setLeft);
      driveBottomRightButton.whenPressed(webcams::setRight);
      driveTopRightButton.whenPressed(new SetToRobotCentric(driveunbun));
      rotateButtonEleven.whenPressed(new ResetFieldCentric(driveunbun));
    }
   }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new HoodReset(hood),
      new SetSpeedAndAngle(shooter, hood, insideTarmac),
      new KickerAutoStart(kicker, conveyor, shooter),
      new WaitCommand(5), // wait for balls to shoot
      new KickerStop(kicker, conveyor),
      new StopSpeedAndAngle(shooter, hood),
      new DriveRobotCentric(driveunbun, 0, 0.7, 0, 1)
    );
  }

  public void hoodReset() {
    if (!hood.isInitialHomed()) {
      hoodReset.schedule(false); // doesn't allow interrupts
    }
  }

  // space out status frame periods so that status frames don't all come in at once
  private static int nextSlowStatusPeriodMs = Constants.slowStatusPeriodBaseMs;
  private static int nextVerySlowStatusPeriodSparkMs = Constants.verySlowStatusPeriodSparkBaseMs;
  
  public static int nextSlowStatusPeriodMs() {
    if (++nextSlowStatusPeriodMs > Constants.slowStatusPeriodMaxMs) {
      nextSlowStatusPeriodMs = Constants.slowStatusPeriodBaseMs;
    }
    return nextSlowStatusPeriodMs;
  }

  public static int nextVerySlowStatusPeriodSparkMs() {
    nextVerySlowStatusPeriodSparkMs += 10;
    return nextVerySlowStatusPeriodSparkMs;
  }

  // stagger unused status frames from Talon controllers
  public static void staggerTalonStatusFrames(WPI_TalonFX talon) {
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 
			RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
  }
}