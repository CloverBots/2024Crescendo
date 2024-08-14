// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import modulelib.SwerveModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.DriveCommand;

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

    public static final class DriveConstants {
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;

        /**
         * The driving gear ratio for the swerve module (MK4i L3). This is how many
         * times the drive motor has to turn in order for the wheel to make 1 rotation.
         */
        public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);

        /**
         * The driving gear ratio for the Swerve Module (MK4i). This is how many times
         * the turning motor has to turn in order for the module to make 1 full
         * rotation.
         */
        public static final double TURNING_GEAR_RATIO = 150.0 / 7.0;

        // Length of the robot chassis, front to back
        public static final double wheelBase = Units.inchesToMeters(18.5); // 24
        // Width of the robot chassis, left to right
        public static final double trackWidth = Units.inchesToMeters(18.5);

        /**
         * The PHYSICAL maximum speed of the robot, if all motors were running at max
         * power. About 5.5435 m/s
         */
        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = (1 / DRIVE_GEAR_RATIO) * (6380.0 / 60)
                * WHEEL_CIRCUMFERENCE;

        /**
         * The maximum speed of the robot, in meters per second during TeleOp. Use this
         * to limit the speed when using a controller.
         */
        public static final double TELEOP_MAX_SPEED_METERS_PER_SECOND = DriveCommand.slowMode ? 3 : 5; // Max is 5.5435
        public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 3;

        /** Maximum speed for the robot's turning. */
        public static final double teleOpMaxAngularSpeed = 3 * (2 * Math.PI);
        public static final double teleOpNormalAngularSpeed = DriveCommand.slowMode ? 2 * (2 * Math.PI)
                : 0.5 * (2 * Math.PI);
        public static final double teleOpSlowAngularSpeed = 1 * (2 * Math.PI);
        /** The maximum angular acceleration for the robot's turning. */
        public static final double teleOpMaxAngularAccelerationUnitsPerSecond = 1;
        /** The maximum acceleration for the robot's X and Y movement. */
        public static final double teleOpMaxAccelerationMetersPerSecond = 3;

        /**
         * Multiply the output of {@code getSelectedSensorPosition()} by this to get the
         * total distance travelled, in meters, on a swerve module.
         */
        public static final double DRIVE_ENCODER_TO_METERS = (WHEEL_CIRCUMFERENCE / (DRIVE_GEAR_RATIO * 2048.0));

        /**
         * Multiply the output of {@code getSelectedSensorVelocity()} by this to get the
         * current velocity, in meters per second, on a swerve module.
         */
        public static final double DRIVE_ENCODER_VELOCITY_TO_METERS_PER_SECOND = (600.0 * WHEEL_CIRCUMFERENCE)
                / (2048.0 * 60 * DRIVE_GEAR_RATIO);

        public static final double MAX_VOLTAGE = 12;
        public static final double deadband = 0.08;
        public static final int currentLimit = 40;
        public static final double slewRate = 20; // lower number for higher center of mass

        public static final class SwervePID {
            public static final double p = 0.12;
            public static final double i = 0;
            public static final double d = 0.0015;
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
            public static final double p = 4;
            public static final double i = 0;
            public static final double d = 0;
        }

        public static final class RotationPID {
            public static final double p = 2.;
            public static final double i = 0;
            public static final double d = 0;
        }
    }

    public static final class VisonConstants {
        private static final double VISION_TARGET_HEIGHT = 57.13; // AprilTag 4, 7
        private static final double CAMERA_HEIGHT = 8.75; // inches
        private static final double CAMERA_PITCH = 35; // degrees
        public final static VisionConfiguration visionConfiguration = new VisionConfiguration(
                VISION_TARGET_HEIGHT,
                CAMERA_HEIGHT,
                CAMERA_PITCH);
    }

    public static final class PivotConstants {
        public static final int PIVOT_LEAD_MOTOR = 31;
        public static final int PIVOT_FOLLOW_MOTOR = 32;
        public static final int PIVOT_ENCODER = 37;
        public static final int PIVOT_LIMIT_SWITCH = 0; // digital input port

        public final static double PIVOT_LOWER_ENDPOINT = 6; // 7 , 5
        public final static double PIVOT_UPPER_ENDPOINT = 100;
        public static final double CLIMBER_PIVOT_SPEED = 0.5;

        public static final double SHOOTER_PARKED_PIVOT_ANGLE = 10; // PARKED
        public static final double SHOOTER_AMP_PIVOT_ANGLE = 70; // AMP
        public static final double SHOOTER_SPEAKER_PIVOT_ANGLE = 66; // SPEAKER
        public static final double SHOOTER_OVER_STAGE_PIVOT_ANGLE = 60; // OVER STAGE
        public static final double SHOOTER_UNDER_STAGE_PIVOT_ANGLE = 10; // UNDER STAGE
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

        public static final double SHOOTER_AMP_RIGHT_RPM = 700;
        public static final double SHOOTER_AMP_LEFT_RPM = 700;
        public static final double SHOOTER_SPEAKER_RIGHT_RPM = 2500;
        public static final double SHOOTER_SPEAKER_LEFT_RPM = 2000;
        public static final double SHOOTER_OVER_STAGE_RIGHT_RPM = 3000;
        public static final double SHOOTER_OVER_STAGE_LEFT_RPM = 2500;
        public static final double SHOOTER_UNDER_STAGE_RIGHT_RPM = 3000;
        public static final double SHOOTER_UNDER_STAGE_LEFT_RPM = 3000;

    }
}