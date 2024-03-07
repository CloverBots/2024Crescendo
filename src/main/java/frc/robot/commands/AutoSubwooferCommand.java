// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSubwooferCommand extends SequentialCommandGroup {
  /** Creates a new AutoFire. */
  public AutoSubwooferCommand(FeederSubsystem feederSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem) {
    addCommands(
      new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_SPEAKER_PIVOT_ANGLE)),
      new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(RobotContainer.SHOOTER_SPEAKER_LEFT_RPM)),
      new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(RobotContainer.SHOOTER_SPEAKER_RIGHT_RPM))
    );
  }
}
