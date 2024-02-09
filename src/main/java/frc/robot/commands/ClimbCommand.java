// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.PivotEncoderSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ClimbCommand extends Command {
  private final PivotSubsystem pivotSubsystem;
  private final PivotEncoderSubsystem pivotEncoderSubsystem;
  private final DoubleSupplier leftJoystickY;
  private final double PIVOT_SPEED = 0.2;
  private final int position;
  private int direction;
  private PIDController pivotPIDController;
  /** Creates a new PivotCommand. */
  public ClimbCommand(PivotSubsystem pivotSubsystem, 
                      PivotEncoderSubsystem pivotEncoderSubsystem, 
                      DoubleSupplier leftJoystickY, 
                      int position, int direction) {
    this.pivotSubsystem = pivotSubsystem;
    this.pivotEncoderSubsystem = pivotEncoderSubsystem;
    this.leftJoystickY = leftJoystickY;
    this.position = position;
    this.direction = direction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivotSubsystem);
    addRequirements(pivotEncoderSubsystem);

    if (position > pivotEncoderSubsystem.PIVOT_UPPER_ENDPOINT) {
      position = (int) pivotEncoderSubsystem.PIVOT_UPPER_ENDPOINT;
    }
    if (position < pivotEncoderSubsystem.PIVOT_LOWER_ENDPOINT) {
      position = (int) pivotEncoderSubsystem.PIVOT_LOWER_ENDPOINT;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    direction = 1; // going up
    if (pivotEncoderSubsystem.getPivotAbsolutePosition() > position) {
      direction = -1; // going down
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pivotSpeed = -leftJoystickY.getAsDouble() * 0.2;

    if (Math.abs(pivotSpeed) <0.05) {
      pivotSpeed = 0;
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
    if (pivotEncoderSubsystem.getPivotAbsolutePosition() > pivotEncoderSubsystem.PIVOT_UPPER_ENDPOINT){
      pivotSubsystem.stop();
    }

    if (pivotEncoderSubsystem.getPivotAbsolutePosition() < pivotEncoderSubsystem.PIVOT_LOWER_ENDPOINT){
      pivotSubsystem.stop();
    }

    return false;
  }
}
