// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoOneBlue extends SequentialCommandGroup {
  /** Creates a new Auto. */
  public AutoOneBlue(SwerveSubsystem swerveSubsystem) {
    addCommands(
        new ResetOdometryCommand(swerveSubsystem),
        // Shoot ring
        new WaitCommand(1),

        // Get 3
        new DriveToDistanceCommand(swerveSubsystem, -2, 0, 0, 10.0)//-57, 60, -20

        // Shoot Position
        // new DriveToDistanceCommand(swerveSubsystem, 0, 0, 0, 10.0)

        // Shoot ring
        // new WaitCommand(1),

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
        // new WaitCommand(1)
        );
  }
}