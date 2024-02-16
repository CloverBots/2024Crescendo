// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class ClimbCommand extends Command {

  private final PivotSubsystem pivotSubsystem;
  private final double PIVOT_SPEED = 0.2;
  private int direction;
  private double position;

  /** Creates a new PivotCommand. */
  public ClimbCommand(PivotSubsystem pivotSubsystem,
      double position) {
    this.pivotSubsystem = pivotSubsystem;
    this.position = position;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivotSubsystem);

    if (position > PivotSubsystem.PIVOT_UPPER_ENDPOINT) {
      position = PivotSubsystem.PIVOT_UPPER_ENDPOINT;
    }
    if (position < PivotSubsystem.PIVOT_LOWER_ENDPOINT) {
      position = PivotSubsystem.PIVOT_LOWER_ENDPOINT;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    direction = 1; // going up
    if (pivotSubsystem.getPivotAbsolutePosition() > position) {
      direction = -1; // going down
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSubsystem.setSpeed(PIVOT_SPEED * direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((direction == 1 && pivotSubsystem.getPivotAbsolutePosition() > position)
        || (direction == -1 && pivotSubsystem.getPivotAbsolutePosition() < position)) {
      pivotSubsystem.stop();
      return true;
    } else {
      return false;
    }
  }
}
