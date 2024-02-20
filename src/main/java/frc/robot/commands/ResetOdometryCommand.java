// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Resets the odometer on the {@link SwerveSubsystem}.
*/
public class ResetOdometryCommand extends Command {

  SwerveSubsystem swerveSubsystem;
  Pose2d pose;
  
  /** Creates a new ResetOdometryCommand. */
  public ResetOdometryCommand(SwerveSubsystem swerveSubsystem, Pose2d pose) {
    addRequirements(swerveSubsystem);
    this.swerveSubsystem = swerveSubsystem;
    this.pose = pose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsystem.resetOdometryPose(pose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
