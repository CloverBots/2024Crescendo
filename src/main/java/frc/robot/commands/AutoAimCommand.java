// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionTargetTracker;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAimCommand extends Command {

  private PIDController lockToTagXController = new PIDController(0.075, 0.03, 0.005);
  private SwerveSubsystem swerve;
  private PivotSubsystem pivotSubsystem;
  private VisionTargetTracker visionTargetTracker;
  private ShooterSubsystem shooterSubsystem;
  private double pivotAngle;
  private double previousPivotAngle;

  public AutoAimCommand(SwerveSubsystem swerve, VisionTargetTracker visionTargetTracker, PivotSubsystem pivotSubsystem,
      ShooterSubsystem shooterSubsystem) {
    addRequirements(swerve);
    this.swerve = swerve;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.lockToTagXController = new PIDController(0.075, 0.03, 0.005);
    this.lockToTagXController.setTolerance(0.05);
    this.visionTargetTracker = visionTargetTracker;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lockToTagXController.reset();
    pivotAngle = pivotSubsystem.getSetpoint();
    previousPivotAngle = pivotAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.setSpeed(0, 0, lockToTagXController.calculate(visionTargetTracker.getTx(), 0), true);

    double targetDistance = 0;
    Boolean isTargetValid = visionTargetTracker.isValid();
    if (isTargetValid) {
      targetDistance = visionTargetTracker.computeTargetDistance();
      pivotAngle = visionTargetTracker.computePivotAngle(targetDistance);
      pivotAngle = checkAngleLimits(pivotAngle);
    }

    // If the desired angle has changed by 1 degree or more, update the setpoint
    if (Math.abs(previousPivotAngle - pivotAngle) > 1) {
      previousPivotAngle = pivotAngle;
      pivotSubsystem.setPivotControllerSetpoint(pivotAngle);

      shooterSubsystem.setShooterLeftRPM(visionTargetTracker.computeShooterLeftSpeed(targetDistance));
      shooterSubsystem.setShooterRightRPM(visionTargetTracker.computeShooterRightSpeed(targetDistance));
    }
  }

  private double checkAngleLimits(double angle) {
    if (angle > PivotSubsystem.PIVOT_UPPER_ENDPOINT) {
      angle = PivotSubsystem.PIVOT_UPPER_ENDPOINT;
    }
    if (angle < PivotSubsystem.PIVOT_LOWER_ENDPOINT) {
      angle = PivotSubsystem.PIVOT_LOWER_ENDPOINT;
    }

    return angle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!visionTargetTracker.isValid()) {
      return true;
    }

    return lockToTagXController.atSetpoint() && pivotSubsystem.atSetpoint();
  }
}
