// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.RabbitDeploySubsystem;
import frc.robot.subsystems.RabbitIntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRabbit extends SequentialCommandGroup {
  /** Creates a new AutoTest. */
  public AutoRabbit(SwerveSubsystem swerveSubsystem, RabbitDeploySubsystem rabbitDeploySubsystem, RabbitIntakeSubsystem rabbitIntakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetOdometryCommand(swerveSubsystem),
      new DriveToDistanceCommand(swerveSubsystem, -Units.inchesToMeters(250), 0, 0, 3.2)
        .alongWith(
          new InstantCommand(() -> rabbitIntakeSubsystem.startIntake(), rabbitIntakeSubsystem),
          new RabbitDeployCommand(rabbitDeploySubsystem, -18.1).raceWith(new WaitCommand(1))
            .andThen(new WaitCommand(.5))
        ),
      
      new DriveToDistanceCommand(swerveSubsystem, 0, 0, Math.PI, 3.2).alongWith(
        new InstantCommand(() -> rabbitIntakeSubsystem.startIntake(0), rabbitIntakeSubsystem),
        new RabbitDeployCommand(rabbitDeploySubsystem, 0).raceWith(new WaitCommand(1))
        ),
      new InstantCommand(() -> rabbitIntakeSubsystem.startIntake(-1), rabbitIntakeSubsystem)
      );
  }
}
