// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionTargetTracker;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAimCommand extends Command {

  private PIDController lockToTagXController = new PIDController(0.075, 0.03, 0.005);
  private SwerveSubsystem swerve;
  private VisionTargetTracker vision;

  public AutoAimCommand(SwerveSubsystem swerve, VisionTargetTracker vision) {
    addRequirements(swerve);
    this.lockToTagXController = new PIDController(0.075, 0.03, 0.005);
    this.lockToTagXController.setTolerance(0.05);
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lockToTagXController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.setSpeed(0, 0, lockToTagXController.calculate(vision.getTx(), 0), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lockToTagXController.atSetpoint();
  }
}
