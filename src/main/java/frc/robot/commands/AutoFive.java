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

// Shoots 3 notes (pre-loaded, 2 close notes)

public class AutoFive extends SequentialCommandGroup {
    /** Creates a new Auto. */
    public AutoFive(SwerveSubsystem swerveSubsystem, FeederSubsystem feederSubsystem, PivotSubsystem pivotSubsystem,
            ShooterSubsystem shooterSubsystem, FeederDistanceSensorSubsystem feederDistanceSensorSubsystem,
            IntakeSubsystem intakeSubsystem, VisionTargetTracker visionTargetTracker) {

        addCommands(
                new ResetOdometryCommand(swerveSubsystem,
                        new Pose2d(new Translation2d(), new Rotation2d())),
                // Shoot ring
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(2000), shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(2500), shooterSubsystem),
                new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(66), pivotSubsystem),

                new WaitCommand(0.8),

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
                        new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-58), Units.inchesToMeters(46),
                                -30, 1.7, false), // -50, 38, -30, 1.5
                        new AutoIntakeCommand(feederDistanceSensorSubsystem, feederSubsystem, intakeSubsystem, 1.7)),
                 
                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-57), Units.inchesToMeters(44), -30,
                        1.0, true), // -50, 38, -30, 1.0
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(2500), shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(2500), shooterSubsystem),
                new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(45), pivotSubsystem),

                new AutoAimCommand(swerveSubsystem, visionTargetTracker, pivotSubsystem, shooterSubsystem, 1.5f),

                new InstantCommand(() -> feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT), feederSubsystem),
                new WaitCommand(0.2),
                new InstantCommand(() -> feederSubsystem.setSpeed(0), feederSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(0), shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(0), shooterSubsystem),
                new InstantCommand(
                        () -> pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE),
                        pivotSubsystem),
                
                new ParallelCommandGroup(
                        new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-74),
                                Units.inchesToMeters(0), 75, 1.5, false), // -67, 0, 75, 1.5
                        new AutoIntakeCommand(feederDistanceSensorSubsystem, feederSubsystem, intakeSubsystem, 1.5)),

                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-74), Units.inchesToMeters(0), 0,
                        1.5, true), // -67, 0, 0, 1.5
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(1500), shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(1500), shooterSubsystem),
                new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(40), pivotSubsystem),

                new AutoAimCommand(swerveSubsystem, visionTargetTracker, pivotSubsystem, shooterSubsystem, 1.0f),

                new InstantCommand(() -> feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT), feederSubsystem),
                new WaitCommand(0.2),
                new InstantCommand(() -> feederSubsystem.setSpeed(0), feederSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(0), shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(0), shooterSubsystem),
                new InstantCommand(
                        () -> pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE),
                        pivotSubsystem),

                 new ParallelCommandGroup(
                        new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-74),
                                Units.inchesToMeters(-54), 75, 1.5, false), // -67, -45, 75, 1.5
                        new AutoIntakeCommand(feederDistanceSensorSubsystem, feederSubsystem, intakeSubsystem, 1.5)),
                
                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-74), Units.inchesToMeters(-54), 22,
                        1.0, true), // -67, -45, 30, 1.0
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(1500), shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(1500), shooterSubsystem),
                new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(40), pivotSubsystem),

                new AutoAimCommand(swerveSubsystem, visionTargetTracker, pivotSubsystem, shooterSubsystem, 1.5f),

                new InstantCommand(() -> feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT), feederSubsystem),
                new WaitCommand(0.2),
                new InstantCommand(() -> feederSubsystem.setSpeed(0), feederSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(0), shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(0), shooterSubsystem),
                new InstantCommand(
                        () -> pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE),
                        pivotSubsystem)
                );
    }
}