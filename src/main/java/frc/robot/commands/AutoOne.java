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

// Shoots 3 notes (pre-loaded, 2 close notes)

public class AutoOne extends SequentialCommandGroup {
    /** Creates a new Auto. */
    public AutoOne(SwerveSubsystem swerveSubsystem, FeederSubsystem feederSubsystem, PivotSubsystem pivotSubsystem,
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
                        new Pose2d(new Translation2d(), new Rotation2d(Units.degreesToRadians(-60 * inverted)))),
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
                new InstantCommand(
                        () -> pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE),
                        pivotSubsystem),
                new ParallelCommandGroup(
                        new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-66), Units.inchesToMeters(15),
                                0, 2.5, false), // -66, 15, 0, 2.5, false
                        new AutoIntakeCommand(feederDistanceSensorSubsystem, feederSubsystem, intakeSubsystem, 2.5)),

                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-66), Units.inchesToMeters(3), -30,
                        1.0, true), // -66, 3, -30, 1.0, true 
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(1500), shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(1500), shooterSubsystem),
                new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(45), pivotSubsystem),

                new AutoAimCommand(swerveSubsystem, visionTargetTracker, pivotSubsystem, shooterSubsystem, 2.0f),

                new WaitCommand(0.5),

                new InstantCommand(() -> feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT), feederSubsystem),
                new WaitCommand(0.2),
                new InstantCommand(() -> feederSubsystem.setSpeed(0), feederSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(0), shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(0), shooterSubsystem),

                new InstantCommand(
                        () -> pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE),
                        pivotSubsystem),
                new ParallelCommandGroup(
                        new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-86),
                                Units.inchesToMeters(-45), 75, 2.0, false), // -86, -45, 75, 2.0, false
                        new AutoIntakeCommand(feederDistanceSensorSubsystem, feederSubsystem, intakeSubsystem, 3)),

                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-86), Units.inchesToMeters(-45), 0,
                        2.0, true), // -86, -45, 0, 2.0, true
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(1500), shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(1500), shooterSubsystem),
                new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(40), pivotSubsystem),

                new AutoAimCommand(swerveSubsystem, visionTargetTracker, pivotSubsystem, shooterSubsystem, 2.0f),

                new WaitCommand(0.5),

                new InstantCommand(() -> feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT), feederSubsystem),
                new WaitCommand(0.2),
                new InstantCommand(() -> feederSubsystem.setSpeed(0), feederSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(0), shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(0), shooterSubsystem),
                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-128), Units.inchesToMeters(-45), 0,
                        2.0, false)); // -128, -45, 0, 2.0, false
    }
}