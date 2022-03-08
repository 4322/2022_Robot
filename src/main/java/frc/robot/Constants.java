// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveDrive.ControlModule.WheelPosition;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final boolean debug = true;

    public static final boolean driveEnabled = false;
    public static final boolean joysticksEnabled = false;
    public static final boolean gyroEnabled = true;
    public static final boolean intakeEnabled = true;
    public static final boolean shooterEnabled = true;
    public static final boolean kickerEnabled = true;
    public static final boolean climberEnabled = false;
    public static final boolean hoodEnabled = true;
    public static final boolean conveyorEnabled = true;
    public static final boolean ultrasonicEnabled = false;

    public static final boolean driveTwoJoystick = true; // defaults to one joystick drive

    public static final int controllerConfigTimeoutMs = 50;

    public static final class DriveConstants {
        public static final int frontRightDriveID = 2;
        public static final int frontRightRotationID = 3;
        public static final int rearRightDriveID = 4;
        public static final int rearRightRotationID = 5;
        public static final int frontLeftDriveID = 6;
        public static final int frontLeftRotationID = 7;
        public static final int rearLeftDriveID = 8;
        public static final int rearLeftRotationID = 9;

        public static final int frontRightEncoderID = 10;
        public static final int rearRightEncoderID = 11;
        public static final int frontLeftEncoderID = 12;
        public static final int rearLeftEncoderID = 13;

        public static final double wheelRadius = 0.0508;
        public static final int encoderResolution = 2048;

        public static final double kMaxSpeed = 3.0;
        public static final double kMaxAngularSpeed = Math.PI;
        public static final double kModuleMaxAngularAcceleration = 2 * Math.PI;

        public static final double distWheelX = 0.339725;
        public static final double distWheelY = 0.244475;

        public static final double wheelBaseLengthFeet = 26.75/12.0;
        public static final double wheelBaseWidthFeet = 19.25/12.0;

        public static final double autoRotkP = 0.015;
        public static final double autoRotkD = 0.0015;

        public static final double maxAutoRotSpd = 0.7;

        public static final double twistDeadband = 0.08;
        public static final double rotateToDeadband = 0.25;

        public final static class Rotation {
            public static final double kP = 1.2;
            public static final double kD = 6.0;

            public static final double configCLosedLoopRamp = 0.08;
            public static final double minPower = 0.0;  // allow for tighter tolerance
            public static final double maxPower = 0.3;  // reduce gear wear and overshoot
            public static final double countToDegrees = 360.0 / encoderResolution * 12 / 24 * 14 / 72;

            public static final double configVoltageCompSaturation = 12;
            public static final boolean enableVoltageCompensation = true;

            public static final boolean statorEnabled = true;
            public static final double statorLimit = 40;
            public static final double statorThreshold = 45;
            public static final double statorTime = 1.0;

            public static final boolean supplyEnabled = true;
            public static final double supplyLimit = 30;
            public static final double supplyThreshold = 35;
            public static final double supplyTime = 0.5;

            public static final double allowableClosedloopError = 0.35 / countToDegrees;

            // values obtained from swerve module zeroing procedure
            public static final double[] CANCoderOffsetDegrees;
            static {
                CANCoderOffsetDegrees = new double[4];
                CANCoderOffsetDegrees[WheelPosition.FRONT_RIGHT.wheelNumber] = 78.311;
                CANCoderOffsetDegrees[WheelPosition.FRONT_LEFT.wheelNumber] = 169.365;
                CANCoderOffsetDegrees[WheelPosition.BACK_RIGHT.wheelNumber] = 106.787;
                CANCoderOffsetDegrees[WheelPosition.BACK_LEFT.wheelNumber] = -104.678;
            }
        }

        public static final class Drive {
            public static final double configOpenLoopRamp = 0.08;

            public static final double configVoltageCompSaturation = 12;
            public static final boolean enableVoltageCompensation = true;

            public static final boolean statorEnabled = true;
            public static final double statorLimit = 40;
            public static final double statorThreshold = 45;
            public static final double statorTime = 1.0;

            public static final boolean supplyEnabled = true;
            public static final double supplyLimit = 40;
            public static final double supplyThreshold = 45;
            public static final double supplyTime = 0.5;

            public static final double wheelDiameter = 4.0;
            public static final int ticksPerRev = 4096;
        }
    }

    public static final class ShooterConstants {

        public static final boolean shooterEnabled = true;

        public static final int flywheelLeftID = 14;
        public static final int flywheelRightID = 15;

        public static final double kP = .00025;
        public static final double kI = 0.000001;
        public static final double kD = 0.004;
        public static final double kIz = 300;
        public static final double kFF = 0.00015;
        public static final double kMinRange = 0;   // let flywheel coast down, don't apply power to slow it
        public static final double kMaxRange = 1.0;
        public static final double rampRate = 1.0;  // seconds to go from stopped to full power

        public static final double minEjectVel = 500;
        
    }

    public static final class KickerConstants {

        public static double kickerPower = 0.5;
        public static final int kickerID = 16;
        public static double rampRate = 0.2;

    }
    
    public static final class HoodConstants {
        // All Constants values need to be adjusted from last years code
        public static final int motorID = 17;
        public static final int hoodMaxPosition = 9300; 
        public static int hoodMinPosition = 0;     //allows to be changed in demo mode
        public static final int hoodDecellerationDistance = 500; 
        public static final int hoodTolerance = 20;
        public static final double homingTimeout = 5.0; 
        public static final double autoTimeout = 3.0; 

        public static double maxForwardPower = 1.0;    //allows to be changed in demo mode
        public static double maxReversePower = -1.0;   //allows to be changed in demo mode
        public static final double minForwardPower = 0.1;
        public static final double minReversePower = -0.1;
        public static final double homingPower = -0.4;
        public static final double manualDeadband = 0.05;

        public static class PID_Values {
            public static final double kP = 3.0;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        public static class Positions {
            //For preset positions
        }
    }

    public static final class IntakeConstants {

        public static final int motorID = 18;
        public static final double intakeSpeed = 0.2;
        public static double rampRate = 0.5;
        
    }

    public static final class ConveyorConstants {

        public static int motorID = 19;
        public static double minBallDistIn = 2;
        public static double conveyorPower = 0.5;

    }
}
