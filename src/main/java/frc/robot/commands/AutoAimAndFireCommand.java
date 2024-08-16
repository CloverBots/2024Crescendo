//  Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.RobotContainer;
import frc.robot.VisionTargetTracker;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAimAndFireCommand extends Command {
  private SwerveSubsystem swerveSubsystem;
  private PivotSubsystem pivotSubsystem;
  private VisionTargetTracker visionTargetTracker;
  private ShooterSubsystem shooterSubsystem;
  private FeederSubsystem feederSubsystem;
  private double pivotAngle;
  private double previousPivotAngle;
  private double time;
  private Timer timer;
  private Timer shotTimer;
  public boolean isFiring = false;

  public AutoAimAndFireCommand(SwerveSubsystem swerveSubsystem, VisionTargetTracker visionTargetTracker, PivotSubsystem pivotSubsystem,
      ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, double time) {
    this.swerveSubsystem = swerveSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.visionTargetTracker = visionTargetTracker;
    this.feederSubsystem = feederSubsystem;
    this.time = time;
    timer = new Timer();
    shotTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    shotTimer.reset();
    pivotAngle = pivotSubsystem.getSetpoint();
    previousPivotAngle = pivotAngle;
    timer.start();
    swerveSubsystem.setRotationOverride(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     if (RobotContainer.getAlliance() == Alliance.Blue) {
      swerveSubsystem.setRotationTarget(Rotation2d.fromDegrees(visionTargetTracker.getAngleToSpeaker()));
    } else {
      swerveSubsystem.setRotationTarget(Rotation2d.fromDegrees(visionTargetTracker.getAngleToSpeaker()).rotateBy(Rotation2d.fromDegrees(180)));
    }
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
    pivotSubsystem.setPivotControllerSetpoint(PivotConstants.PIVOT_PARKED_ANGLE);
    shooterSubsystem.setDefaultShooterRPM();
    feederSubsystem.setSpeed(0);
    timer.stop();
    shotTimer.stop();
    swerveSubsystem.setRotationOverride(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!isFiring && shooterSubsystem.isShooterAtTargetRpm() && pivotSubsystem.pivotReady() && visionTargetTracker.getAngleToSpeaker() < 3 || timer.get() > time) {
      feederSubsystem.setSpeed(IntakeConstants.FEEDER_SPEED_SHOOT);
      isFiring = true;
      shotTimer.start();
    }

    if (isFiring && shotTimer.get() > 0.5) {
      return true;
    } else {
      return false;
    }

  }
}