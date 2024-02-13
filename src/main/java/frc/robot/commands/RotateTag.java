// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.VisionTargetTracker;

public class RotateTag extends Command {

  SwerveSubsystem swerveSubsystem;
  PIDController rotationController;
  VisionTargetTracker limelightVision;

  /** Creates a new RotateTag. */
  public RotateTag(SwerveSubsystem swerveSubsystem, VisionTargetTracker limelightVision) {
    this.swerveSubsystem = swerveSubsystem;
    rotationController = new PIDController(0.5, 0, 0);
    this.limelightVision = limelightVision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = rotationController.calculate(limelightVision.getTx());
    swerveSubsystem.setSpeed(0, 0, speed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
