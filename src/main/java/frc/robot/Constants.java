// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import modulelib.SwerveModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import limelight.LimelightConfiguration;

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
    public static final int CONTROLLER_DRIVE_PORT = 0;
    public static final int CONTROLLER_OPERATOR_PORT = 1;
    public static final int LED_ID = 2;
    public static final int CURRENT_LIMIT = 100;

    public static final class DriveConstants {
        // All for MK4i modules
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;
        public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
        public static final double TURNING_GEAR_RATIO = 150.0 / 7.0;
        public static final double FALCON_FREE_SPEED = 6380.0;
        public static final double NEO_FREE_SPEED = 5820.0;;
        public static final double wheelBase = Units.inchesToMeters(18.5);
        public static final double trackWidth = Units.inchesToMeters(18.5);
        
        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = (1 / DRIVE_GEAR_RATIO) * (FALCON_FREE_SPEED / 60)
                * WHEEL_CIRCUMFERENCE; // Max is 5.5435

        public static final double PHYSICAL_MAX_ROTATION_SPEED = (1 / TURNING_GEAR_RATIO) * (NEO_FREE_SPEED / 60)
                * WHEEL_CIRCUMFERENCE; // Max is 1.4448

        // Speeds for the robot when moving, in Meters/Second
        public static final double TELEOP_MAX_SPEED_METERS_PER_SECOND = 5;
        public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 3;

        // Rotation speed multiplier to the (-1, 1) input given by the joystick
        public static final double TELEOP_NORMAL_ANGULAR_SCALE_FACTOR = 0.8;
        public static final double TELEOP_SLOW_ANGULAR_SCALE_FACTOR = 0.4;

        public static final double DRIVE_ENCODER_TO_METERS = (WHEEL_CIRCUMFERENCE / (DRIVE_GEAR_RATIO * 2048.0));
        public static final double DRIVE_ENCODER_VELOCITY_TO_METERS_PER_SECOND = (600.0 * WHEEL_CIRCUMFERENCE)
                / (2048.0 * 60 * DRIVE_GEAR_RATIO);

        public static final double MAX_VOLTAGE = 6;
        public static final double deadband = 0.08;
        public static final int currentLimit = 40;
        public static final double slewRate = 50; // lower number for higher center of mass

        public static final class SwervePID {
            public static final double p = 0.15;
            public static final double i = 0;
            public static final double d = 0;
        }

        public static final class SwerveModules {
            public static final SwerveModuleConfig frontRight = new SwerveModuleConfig(11, 15, 19, false);
            public static final SwerveModuleConfig frontLeft = new SwerveModuleConfig(10, 14, 18, true);
            public static final SwerveModuleConfig backLeft = new SwerveModuleConfig(13, 17, 21, true);
            public static final SwerveModuleConfig backRight = new SwerveModuleConfig(12, 16, 20, false);
        }

        public static final class ModuleLocations {
            public static final double dist = Units.inchesToMeters(9.25);
            public static final double robotRaduius = Math.sqrt(2 * Math.pow(dist, 2));
            public static final Translation2d frontLeft = new Translation2d(dist, dist);
            public static final Translation2d frontRight = new Translation2d(dist, -dist);
            public static final Translation2d backLeft = new Translation2d(-dist, dist);
            public static final Translation2d backRight = new Translation2d(-dist, -dist);
        }
    }

    public static final class PathPlannerConstants {
        public static final class TranslationPID {
            public static final double p = 2.5; // 4 (Non carpet); 3 (Carpet)
            public static final double i = 0;
            public static final double d = 0;
        }

        public static final class RotationPID {
            public static final double p = 2.9; // 2 (Non carpet); 4 (Carpet)
            public static final double i = 0;
            public static final double d = 0;
        }
    }

    public static final class VisonConstants {
        private static final double VISION_TARGET_HEIGHT = 57.13; // AprilTag 4, 7
        private static final double CAMERA_HEIGHT = 8.75; // inches
        private static final double CAMERA_PITCH = 35; // degrees
        public final static LimelightConfiguration visionConfiguration = new LimelightConfiguration(
                VISION_TARGET_HEIGHT,
                CAMERA_HEIGHT,
                CAMERA_PITCH);
    }

    public static final class PivotConstants {
        public static final int PIVOT_LEAD_MOTOR = 31;
        public static final int PIVOT_FOLLOW_MOTOR = 32;
        public static final int PIVOT_ENCODER = 37;
        public static final int PIVOT_LIMIT_SWITCH = 0; // digital input port

        public final static double PIVOT_PHYSICAL_LOWER_ENDPOINT = 1;
        public final static double PIVOT_LOWER_ENDPOINT = 2;
        public final static double PIVOT_UPPER_ENDPOINT = 100;
        public static final double CLIMBER_PIVOT_SPEED = 0.5;
        public final static double MAX_PIVOT_POWER_PID = 0.6;

        public static final double PIVOT_PARKED_ANGLE = 10; // PARKED
        public static final double PIVOT_AMP_ANGLE = 70; // AMP
        public static final double PIVOT_SPEAKER_ANGLE = 66; // SPEAKER
        public static final double PIVOT_OVER_STAGE_ANGLE = 60; // OVER STAGE
        public static final double PIVOT_UNDER_STAGE_ANGLE = 10; // UNDER STAGE
        public static final double PIVOT_LINE_ANGLE = 54; // LINE (Auto)
        public static final double PIVOT_FAR_ANGLE = 30.5; // FAR (Auto)
    }

    public static final class IntakeConstants {
        public static final int FEEDER_MOTOR = 30;
        public static final int INTAKE_MOTOR_ID = 33;
        public static final int CENTER_MOTOR_1 = 36;
        public static final int CENTER_MOTOR_2 = 46;

        public static final double INTAKE_SPEED = 1;
        public static final double FEEDER_SPEED_INTAKE = 0.5;
        public static final double FEEDER_SPEED_SHOOT = 0.8;
        public final static double FEEDER_TIME = 1;
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_LEFT_MOTOR_ID = 35;
        public static final int SHOOTER_RIGHT_MOTOR_ID = 34;

        public static final double SHOOTER_AMP_RIGHT_RPM = 500;
        public static final double SHOOTER_AMP_LEFT_RPM = 500;
        public static final double SHOOTER_SPEAKER_RIGHT_RPM = 2500;
        public static final double SHOOTER_SPEAKER_LEFT_RPM = 2000;
        public static final double SHOOTER_OVER_STAGE_RIGHT_RPM = 3000;
        public static final double SHOOTER_OVER_STAGE_LEFT_RPM = 2500;
        public static final double SHOOTER_UNDER_STAGE_RIGHT_RPM = 3000;
        public static final double SHOOTER_UNDER_STAGE_LEFT_RPM = 3000;
        public static final double SHOOTER_LINE_RIGHT_RPM = 3000;
        public static final double SHOOTER_LINE_LEFT_RPM = 2500;
        public static final double SHOOTER_FAR_LEFT_RPM = 4000;
        public static final double SHOOTER_FAR_RIGHT_RPM = 3300;
    }
}