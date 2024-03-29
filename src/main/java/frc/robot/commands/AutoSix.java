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

public class AutoSix extends SequentialCommandGroup {
    /** Creates a new Auto. */
    public AutoSix(SwerveSubsystem swerveSubsystem, FeederSubsystem feederSubsystem, PivotSubsystem pivotSubsystem,
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
                                new Rotation2d())),

                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-90), Units.inchesToMeters(-8), 0,
                        1.0, false),

                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(3000),
                        shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(3000),
                        shooterSubsystem),
                new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(30),
                        pivotSubsystem),

                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-100), Units.inchesToMeters(40), 20,
                        1.0, false),

                new AutoAimCommand(swerveSubsystem, visionTargetTracker, pivotSubsystem,
                        shooterSubsystem, 1.0f),
                new InstantCommand(() -> feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT), feederSubsystem),
                new WaitCommand(0.2),
                new InstantCommand(() -> feederSubsystem.setSpeed(0), feederSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(0),
                        shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(0),
                        shooterSubsystem),
                new InstantCommand(
                        () -> pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE),
                        pivotSubsystem),

                new ParallelCommandGroup(
                        new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-12 * 24 - 6),
                                Units.inchesToMeters(6),
                                0, 2.5, false), // -50, 38, -30, 1.5
                        new AutoIntakeCommand(feederDistanceSensorSubsystem, feederSubsystem,
                                intakeSubsystem, 2.5)),

                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(3000),
                        shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(3000),
                        shooterSubsystem),
                new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(30),
                        pivotSubsystem),

                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-100), Units.inchesToMeters(40), 25,
                        2.0, false),

                new AutoAimCommand(swerveSubsystem, visionTargetTracker, pivotSubsystem,
                        shooterSubsystem, 1.5f),

                new InstantCommand(() -> feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT), feederSubsystem),
                new WaitCommand(0.2),
                new InstantCommand(() -> feederSubsystem.setSpeed(0), feederSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(0),
                        shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(0),
                        shooterSubsystem),
                new InstantCommand(
                        () -> pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE),
                        pivotSubsystem),

                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-12 * 14 - 6),
                        Units.inchesToMeters(40), 0,
                        1.0, false),

                new ParallelCommandGroup(
                        new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-12 * 23 - 6),
                                Units.inchesToMeters(84),
                                0, 2.5, false), // -50, 38, -30, 1.5
                        new AutoIntakeCommand(feederDistanceSensorSubsystem, feederSubsystem,
                                intakeSubsystem, 2.5)),

                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-12 * 14 - 6),
                        Units.inchesToMeters(40), 0,
                        1.0, false),

                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(3000),
                        shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(3000),
                        shooterSubsystem),
                new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(30),
                        pivotSubsystem),

                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-100), Units.inchesToMeters(40), 25,
                        2.0, false),

                new AutoAimCommand(swerveSubsystem, visionTargetTracker, pivotSubsystem,
                        shooterSubsystem, 1.0f),

                new InstantCommand(() -> feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT), feederSubsystem),
                new WaitCommand(0.2),
                new InstantCommand(() -> feederSubsystem.setSpeed(0), feederSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(0),
                        shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(0),
                        shooterSubsystem),
                new InstantCommand(
                        () -> pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE),
                        pivotSubsystem)

        );
    }
}