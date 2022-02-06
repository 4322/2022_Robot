// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

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
    public static final boolean driveEnabled = true;
    public static final boolean intakeEnabled = false;
    public static final boolean shooterEnabled = false;
    public static final boolean climberEnabled = false;

    public final class DriveConstants {
        public static final int frontRightDriveID = 2;
        public static final int frontRightRotationID = 3;
        public static final int rearRightDriveID = 4;
        public static final int rearRightRotationID = 5;
        public static final int frontLeftDriveID = 6;
        public static final int frontLeftRotationID = 7;
        public static final int rearLeftDriveID = 8;
        public static final int rearLeftRotationID = 9;

        public static final double kWheelRadius = 0.0508;
        public static final int kEncoderResolution = 4096;

        public static final double kMaxSpeed = 3.0;
        public static final double kMaxAngularSpeed = Math.PI;
        public static final double kModuleMaxAngularAcceleration = 2 * Math.PI;

        public static final double distWheelX = 0.339725;
        public static final double distWheelY = 0.244475;

        public final class SwerveModuleConstants {

            // Nothing for now

        }


        public final class Rotation {
            public static final double kP = 0.007;
            public static final double kD = 0.00024;

            public static final double configOpenLoopRamp = 0.08;
            public static final double configCLosedLoopRamp = 0.08;

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
        }

        public final class Drive {
            public static final double kP = 0.06;
            public static final double kI = 0.0;
            public static final double kIZone = 0.0;
            public static final double kD = 0.0;
            public static final double kF = 0.083;

            public static final double configOpenLoopRamp = 0.08;
            public static final double configCLosedLoopRamp = 0.08;

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
        }
    }
}
