// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PivotSubsystem;

public class ClimbManualCommand extends Command {
  private final PivotSubsystem pivotSubsystem;
  private final DoubleSupplier leftJoystickY;

  /** Creates a new PivotCommand. */
  public ClimbManualCommand(PivotSubsystem pivotSubsystem,
      DoubleSupplier leftJoystickY) {
    this.pivotSubsystem = pivotSubsystem;
    this.leftJoystickY = leftJoystickY;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(leftJoystickY.getAsDouble()) > .05) {
      double pivotSpeed = leftJoystickY.getAsDouble() / 3; //TO-DO remove the 3 or adjust it
      pivotSpeed = checkSpeed(pivotSpeed);
      pivotSubsystem.setSpeed(pivotSpeed); 
      
    } else {
      pivotSubsystem.setSpeed(0);
    }
  }

  private double checkSpeed(double speed) {
  if (leftJoystickY.getAsDouble() < -.05 &&
        pivotSubsystem.getPivotAbsolutePosition() > RobotContainer.PIVOT_UPPER_ENDPOINT) {
          System.out.println("Stopping climber");
        speed = 0;
    }

    if (leftJoystickY.getAsDouble() > .05 &&
        pivotSubsystem.getPivotAbsolutePosition() < RobotContainer.PIVOT_LOWER_ENDPOINT) {
      speed = 0;
    }

    return speed;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    

    return false;
  }
}
