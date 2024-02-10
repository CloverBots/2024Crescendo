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

public class ClimbCommand extends Command {

  private final PivotSubsystem pivotSubsystem;
  private final PivotEncoderSubsystem pivotEncoderSubsystem;
  private final double PIVOT_SPEED = 0.2;
  private int direction;
  private double position;
  private PIDController pivotPIDController;
  /** Creates a new PivotCommand. */
  public ClimbCommand(PivotSubsystem pivotSubsystem, 
                      PivotEncoderSubsystem pivotEncoderSubsystem, 
                      double position) {
    this.pivotSubsystem = pivotSubsystem;
    this.pivotEncoderSubsystem = pivotEncoderSubsystem;
    this.position = position;


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
    if (pivotSubsystem.getOwner().equals("")) {
      pivotSubsystem.setOwner(this.getName());
    }
    if (pivotSubsystem.getOwner().equals(this.getName())) {
      pivotSubsystem.setSpeed(PIVOT_SPEED * direction);
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
    if ((direction == 1 && pivotEncoderSubsystem.getPivotAbsolutePosition() > position)
    || (direction == -1 && pivotEncoderSubsystem.getPivotAbsolutePosition() < position)){
      pivotSubsystem.stop();
      pivotSubsystem.setOwner("");
      return true;
    } else {
      return false;
    }
  }
}
