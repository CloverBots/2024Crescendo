// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFourBlue extends SequentialCommandGroup {
  /** Creates a new Auto. */
  public AutoFourBlue(SwerveSubsystem swerveSubsystem) {
    addCommands(
        new ResetOdometryCommand(swerveSubsystem, new Pose2d()),
        // Shoot ring
        new WaitCommand(1),

        // leave
        new DriveToDistanceCommand(swerveSubsystem, -Units.inchesToMeters(0), -Units.inchesToMeters(0), Math.PI, 1.5),

        // Wait
        new WaitCommand(1));
  }
}