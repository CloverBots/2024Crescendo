// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

// Shoots 2 notes (pre-load and distant one)

public class AutoSeven extends SequentialCommandGroup {

    public AutoSeven(SwerveSubsystem swerveSubsystem, FeederSubsystem feederSubsystem, PivotSubsystem pivotSubsystem,
            ShooterSubsystem shooterSubsystem, FeederDistanceSensorSubsystem feederDistanceSensorSubsystem,
            IntakeSubsystem intakeSubsystem, VisionTargetTracker visionTargetTracker) {

        Optional<Alliance> side = DriverStation.getAlliance();
        int inverted = 1; // default Blue

        if (side.isPresent()) {
            if (side.get() == Alliance.Red) {
                inverted = -1;
            }
        }
        addCommands(
                new ResetOdometryCommand(swerveSubsystem,
                        new Pose2d(new Translation2d(),
                                new Rotation2d(Units.degreesToRadians(-60 * inverted)))),
                // Shoot ring
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(3000),
                        shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(3000),
                        shooterSubsystem),
                new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(40),
                        pivotSubsystem),

                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-64),
                        Units.inchesToMeters(72), -55, 1.5, false), // -64, 72, -60, 1.5, false

                new AutoAimCommand(swerveSubsystem, visionTargetTracker, pivotSubsystem,
                        shooterSubsystem, 1.0f),

                new InstantCommand(() -> feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT),
                        feederSubsystem),
                new WaitCommand(0.2),
                new InstantCommand(() -> feederSubsystem.setSpeed(0), feederSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(500),
                        shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(500),
                        shooterSubsystem),

                // Get 3
                new InstantCommand(
                        () -> pivotSubsystem.setPivotControllerSetpoint(
                                RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE),
                        pivotSubsystem),

                new ParallelCommandGroup(
                        new DriveToDistanceCommand(swerveSubsystem,
                                Units.inchesToMeters(-12 * 24 - 6),
                                Units.inchesToMeters(157),
                                0, 3.0, false), // -12 * 24 - 6, 157, 0, 4.0, false
                        new AutoIntakeCommand(feederDistanceSensorSubsystem, feederSubsystem,
                                intakeSubsystem, 3)),

                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-12 * 12 - 6),
                        Units.inchesToMeters(130), -50, 1.5, false), // -72, 140, -60, 3.5, false

                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(3000),
                        shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(3000),
                        shooterSubsystem),
                new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(30),
                        pivotSubsystem),

                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-72),
                        Units.inchesToMeters(48), -50, 1.3, false), // -90, 72, -50, 2.0, false

                new AutoAimCommand(swerveSubsystem, visionTargetTracker, pivotSubsystem,
                        shooterSubsystem, 1.0f),

                new InstantCommand(() -> feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT),
                        feederSubsystem),
                new WaitCommand(0.2),
                new InstantCommand(() -> feederSubsystem.setSpeed(0), feederSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(500),
                        shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(500),
                        shooterSubsystem),
                new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE)),

                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-115),
                        Units.inchesToMeters(33),
                        0, 1.0, false),
                
                new WaitCommand(0.1),
                
                new ParallelCommandGroup(
                        new DriveToDistanceCommand(swerveSubsystem,
                                Units.inchesToMeters(-12 * 24 - 6),
                                Units.inchesToMeters(17),
                                0, 2.5, false), // -12 * 24 - 6, 142, 0, 4.0, false
                        new AutoIntakeCommand(feederDistanceSensorSubsystem, feederSubsystem,
                                intakeSubsystem, 2.5)),
                                
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(3500),
                        shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(3500),
                        shooterSubsystem),
                new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(25),
                        pivotSubsystem),

                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-120),
                        Units.inchesToMeters(5),
                        0, 1.5, false),
                    
                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-96),
                        Units.inchesToMeters(-24),
                        -10, 1.0, false),

                new AutoAimCommand(swerveSubsystem, visionTargetTracker, pivotSubsystem,
                        shooterSubsystem, 1.0f),

                new InstantCommand(() -> feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT),
                        feederSubsystem),
                new WaitCommand(0.2),
                new InstantCommand(() -> feederSubsystem.setSpeed(0), feederSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(0),
                        shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(0),
                        shooterSubsystem),
                new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE))
                
                /* new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-166),
                        Units.inchesToMeters(24),
                        0, 2.0, false),
                
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(3500),
                        shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(3500),
                        shooterSubsystem),
                new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(25),
                        pivotSubsystem),

                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-84),
                        Units.inchesToMeters(72), -50, 1.5, false), // -90, 72, -50, 2.0, false
                
                new AutoAimCommand(swerveSubsystem, visionTargetTracker, pivotSubsystem,
                        shooterSubsystem, 1.0f),

                new InstantCommand(() -> feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT),
                        feederSubsystem),
                new WaitCommand(0.2),
                new InstantCommand(() -> feederSubsystem.setSpeed(0), feederSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(0),
                        shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(0),
                        shooterSubsystem),
                new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE)) */
        );
    }
}