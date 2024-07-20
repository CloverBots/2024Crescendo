// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import modulelib.SwerveModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

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
  public static final class DriveConstants {
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;

    /** The driving gear ratio for the swerve module (MK4i L3). This is how many times the drive motor has to turn in order for the wheel to make 1 rotation. */
    public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);

    /** The driving gear ratio for the Swerve Module (MK4i). This is how many times the turning motor has to turn in order for the module to make 1 full rotation. */
    public static final double TURNING_GEAR_RATIO = 150.0/7.0;

    // Length of the robot chassis, front to back
    public static final double wheelBase = Units.inchesToMeters(18.5); //24
    // Width of the robot chassis, left to right
    public static final double trackWidth = Units.inchesToMeters(18.5);
    
    /** The PHYSICAL maximum speed of the robot, if all motors were running at max power. About 5.5435 m/s*/
    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = (1/DRIVE_GEAR_RATIO) * (6380.0/60) * WHEEL_CIRCUMFERENCE;
    
    /** The maximum speed of the robot, in meters per second during TeleOp. Use this to limit the speed when using a controller.*/
    public static final double TELEOP_MAX_SPEED_METERS_PER_SECOND = 0.2; //Max is 5.5435
    public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 0.5;
    
    /** Maximum speed for the robot's turning. */
    public static final double teleOpMaxAngularSpeed = 1 * (2 * Math.PI); // todo change all maxes
    public static final double teleOpNormalAngularSpeed = 1 * (2 * Math.PI);
    public static final double teleOpSlowAngularSpeed = 1 * (2 * Math.PI);
    /** The maximum angular acceleration for the robot's turning. */
    public static final double teleOpMaxAngularAccelerationUnitsPerSecond = 1;
    /** The maximum acceleration for the robot's X and Y movement. */
    public static final double teleOpMaxAccelerationMetersPerSecond = 0.2; //todo change

    /** Multiply the output of {@code getSelectedSensorPosition()} by this to get the total distance travelled, in meters, on a swerve module. */
    public static final double DRIVE_ENCODER_TO_METERS = (WHEEL_CIRCUMFERENCE / (DRIVE_GEAR_RATIO * 2048.0));

    /** Multiply the output of {@code getSelectedSensorVelocity()} by this to get the current velocity, in meters per second, on a swerve module. */
    public static final double DRIVE_ENCODER_VELOCITY_TO_METERS_PER_SECOND = (600.0 * WHEEL_CIRCUMFERENCE) / (2048.0 * 60 * DRIVE_GEAR_RATIO);

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

  public static final class AutoConstants {
    public static final class XPID {
      public static final double p = 1.5;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class YPID {
      public static final double p = 1.5;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class RPID {
      public static final double p = 0.0015;
      public static final double i = 0;
      public static final double d = 0.0002;
    }

    public static final int medianFilter = 5;
  }

  public static final class PathPlannerConstants {
    public static final class TranslationPID {
      public static final double p = 2;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class RotationPID {
      public static final double p = 1.0;
      public static final double i = 0;
      public static final double d = 0;
    }
  }

  public static final class IDS {
    public static final int CONTROLLER_DRIVE_PORT = 0;
    public static final int CONTROLLER_OPERATOR_PORT = 1;
    public static final int FEEDER_MOTOR = 30;
  public static final int PIVOT_LEAD_MOTOR = 31; //32
  public static final int PIVOT_FOLLOW_MOTOR = 32; //31
  public static final int PIVOT_ENCODER = 37;

  public static final int INTAKE_MOTOR_ID = 33;
  public static final int SHOOTER_LEFT_MOTOR_ID = 35;
  public static final int SHOOTER_RIGHT_MOTOR_ID = 34;
  public static final int CENTER_MOTOR_1 = 36;
  public static final int CENTER_MOTOR_2 = 46;

  public static final int PIVOT_LIMIT_SWITCH = 0; //digital input port
  }
}