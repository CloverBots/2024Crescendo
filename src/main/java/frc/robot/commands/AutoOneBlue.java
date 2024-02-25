// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.VisionTargetTracker;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoOneBlue extends SequentialCommandGroup {
  /** Creates a new Auto. */
  public AutoOneBlue(SwerveSubsystem swerveSubsystem, FeederSubsystem feederSubsystem, PivotSubsystem pivotSubsystem,
      ShooterSubsystem shooterSubsystem, FeederDistanceSensorSubsystem feederDistanceSensorSubsystem,
      IntakeSubsystem intakeSubsystem, VisionTargetTracker visionTargetTracker) {
    addCommands(
        new ResetOdometryCommand(swerveSubsystem, new Pose2d(new Translation2d(), new Rotation2d(Units.degreesToRadians(-45)))),
        // Shoot ring
        new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(2000), shooterSubsystem),
        new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(2500), shooterSubsystem),
        new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(65), pivotSubsystem),

        new WaitCommand(1.8),

        new InstantCommand(() -> feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT), feederSubsystem),
        new WaitCommand(0.2),
        new InstantCommand(() -> feederSubsystem.setSpeed(0), feederSubsystem),
        new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(0), shooterSubsystem),
        new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(0), shooterSubsystem),

        // Get 3
        new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE),
            pivotSubsystem),
        new ParallelCommandGroup(
            new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-70), Units.inchesToMeters(-3), 0, 2.5, false),
            new AutoIntakeCommand(feederDistanceSensorSubsystem, feederSubsystem, intakeSubsystem, 2)
        ),

        new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-70), Units.inchesToMeters(-3), -30, 1.0, true),
        new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(1500), shooterSubsystem),
        new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(1500), shooterSubsystem),
        new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(45), pivotSubsystem), 
        
        new AutoAimCommand(swerveSubsystem, visionTargetTracker, pivotSubsystem, shooterSubsystem, 2.0f),

        new WaitCommand(0.1),

        new InstantCommand(() -> feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT), feederSubsystem),
        new WaitCommand(0.2),
        new InstantCommand(() -> feederSubsystem.setSpeed(0), feederSubsystem),
        new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(0), shooterSubsystem),
        new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(0), shooterSubsystem),
 
         new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE),
            pivotSubsystem),
        new ParallelCommandGroup(
            new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-90), Units.inchesToMeters(-45), 90, 2.0, false),
            new AutoIntakeCommand(feederDistanceSensorSubsystem, feederSubsystem, intakeSubsystem, 3)
        ),

        new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-90), Units.inchesToMeters(-45), 0, 2.0, true),
        new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(1500), shooterSubsystem),
        new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(1500), shooterSubsystem),
        new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(45), pivotSubsystem), 
        
        new AutoAimCommand(swerveSubsystem, visionTargetTracker, pivotSubsystem, shooterSubsystem, 2.0f),
        
        new WaitCommand(0.1),

        new InstantCommand(() -> feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT), feederSubsystem),
        new WaitCommand(0.2),
        new InstantCommand(() -> feederSubsystem.setSpeed(0), feederSubsystem),
        new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(0), shooterSubsystem),
        new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(0), shooterSubsystem)
        // Shoot Position
/* 
        new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-61), Units.inchesToMeters(-110), 90, 3.0, false),
        new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-75), Units.inchesToMeters(-120), 0, 3.0, false)
        // Get 8
        // new DriveToDistanceCommand(swerveSubsystem, -Units.inchesToMeters(40), Units.inchesToMeters(100), 0, 10.0),
        // new DriveToDistanceCommand(swerveSubsystem, -Units.inchesToMeters(284), Units.inchesToMeters(200), 0, 10.0) //201

        // Throw Position
        //new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(201), Units.inchesToMeters(180), 0, 10.0),

        // Shoot ring
        //new WaitCommand(1),

        // Get 7
        //new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(135), -Units.inchesToMeters(284), 0, 10.0)

        // Throw Position
        // new DriveToDistanceCommand(swerveSubsystem, 0, 0, 0, 1.5),

        // Shoot ring
        //new WaitCommand(1),

        // Get 6
        // new DriveToDistanceCommand(swerveSubsystem, -Units.inchesToMeters(0), -Units.inchesToMeters(0), 0, 1.5),

        // Throw Position
        // new DriveToDistanceCommand(swerveSubsystem, 0, 0, 0, 1.5),

        // Shoot ring
        // new WaitCommand(1) */
        );
  }
}