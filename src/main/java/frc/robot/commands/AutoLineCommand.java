// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoLineCommand extends ParallelCommandGroup {
  /** Creates a new AutoFire. */
  public AutoLineCommand(FeederSubsystem feederSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem) {
    addCommands(
      new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(55)), // Need line values
      new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(2500)),
      new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(3000))
    );
  }
}
