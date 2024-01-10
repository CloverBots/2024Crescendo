// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoToteCrashRight extends SequentialCommandGroup {
  /** Creates a new Auto2. */
  public AutoToteCrashRight(SwerveSubsystem swerveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(swerveSubsystem);
    addCommands(
      new ResetOdometryCommand(swerveSubsystem),
      new DriveToDistanceCommand(swerveSubsystem, -Units.inchesToMeters(272), 0, 0, 3),
      new DriveToDistanceCommand(swerveSubsystem, -Units.inchesToMeters(272), Units.inchesToMeters(80), 0, 5)
    );
  }
}
