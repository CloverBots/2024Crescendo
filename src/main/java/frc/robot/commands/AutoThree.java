// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

// Shoots pre-loaded note and moves toward the amp
// Uses delayed start option

public class AutoThree extends SequentialCommandGroup {

    public AutoThree(SwerveSubsystem swerveSubsystem, FeederSubsystem feederSubsystem, PivotSubsystem pivotSubsystem,
            ShooterSubsystem shooterSubsystem, FeederDistanceSensorSubsystem feederDistanceSensorSubsystem,
            IntakeSubsystem intakeSubsystem, VisionTargetTracker visionTargetTracker) {

        double wait = SmartDashboard.getNumber("Auto Wait Seconds", 0);

        addCommands(
                new ResetOdometryCommand(swerveSubsystem, new Pose2d()),
                // Shoot ring
                new WaitCommand(wait),
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
                new InstantCommand(
                        () -> pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE),
                        pivotSubsystem),

                // move out of home area
                new DriveToDistanceCommand(swerveSubsystem, Units.inchesToMeters(-70), Units.inchesToMeters(-3), 0, 2.5,
                        false)); // TO-DO proper values
    }
}