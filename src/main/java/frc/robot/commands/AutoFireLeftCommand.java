// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFireLeftCommand extends SequentialCommandGroup {
  /** Creates a new AutoFire. */
  public AutoFireLeftCommand(FeederSubsystem feederSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem) {
    addCommands(
      new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(0)), // Need values
      new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(0)),
      new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(0)),

      new WaitCommand(0.2),

      new InstantCommand(() -> feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT)),
      new WaitCommand(0.2),
      new InstantCommand(() -> feederSubsystem.setSpeed(0)),
      new InstantCommand(() -> pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE)),
      new InstantCommand(() -> shooterSubsystem.setShooterLeftRPM(2000)),
      new InstantCommand(() -> shooterSubsystem.setShooterRightRPM(2000))
    );
  }
}
