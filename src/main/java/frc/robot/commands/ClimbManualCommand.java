// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.PivotEncoderSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ClimbManualCommand extends Command {
  private static final double READY_POSITION = 10;
  private static final double RAISED_POSITION = 15;
  private final PivotSubsystem pivotSubsystem;
  private final PivotEncoderSubsystem pivotEncoderSubsystem;
  private final DoubleSupplier leftJoystickY;
  private final double PIVOT_SPEED = 0.2;
  private int direction;
  private PIDController pivotPIDController;

  /** Creates a new PivotCommand. */
  public ClimbManualCommand(PivotSubsystem pivotSubsystem,
      PivotEncoderSubsystem pivotEncoderSubsystem,
      DoubleSupplier leftJoystickY) {
    this.pivotSubsystem = pivotSubsystem;
    this.pivotEncoderSubsystem = pivotEncoderSubsystem;
    this.leftJoystickY = leftJoystickY;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivotSubsystem);
    addRequirements(pivotEncoderSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(leftJoystickY.getAsDouble()) > .1) {

      if (pivotSubsystem.getOwner().equals("")) {
        pivotSubsystem.setOwner(this.getName());
      }
      if (pivotSubsystem.getOwner().equals(this.getName())) {
        if (Math.abs(leftJoystickY.getAsDouble()) > .1) {
          double pivotSpeed = -leftJoystickY.getAsDouble() * 0.2;
          pivotSubsystem.setSpeed(pivotSpeed);
        } else {
          pivotSubsystem.setSpeed(0);
          pivotSubsystem.setOwner("");
        }
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pivotEncoderSubsystem.getPivotAbsolutePosition() > pivotEncoderSubsystem.PIVOT_UPPER_ENDPOINT) {
      pivotSubsystem.stop();
    }

    if (pivotEncoderSubsystem.getPivotAbsolutePosition() < pivotEncoderSubsystem.PIVOT_LOWER_ENDPOINT) {
      pivotSubsystem.stop();
    }

    return false;
  }
}
