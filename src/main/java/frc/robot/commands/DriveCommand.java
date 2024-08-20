// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import limelight.LimelightTargetTracking;

public class DriveCommand extends Command {

  private final SwerveSubsystem drivebase;
  private LimelightTargetTracking limelight;
  private final Supplier<double[]> speedXY;
  private final DoubleSupplier rot;
  private final double leftTrigger;
  public static boolean lockOnMode = false;

  private PIDController lockToTagXController;

  public DriveCommand(SwerveSubsystem drivebase, Supplier<double[]> speedXY, DoubleSupplier rot,
      double leftTrigger, LimelightTargetTracking limelight) {
    this.drivebase = drivebase;
    this.speedXY = speedXY;
    this.rot = rot;
    this.leftTrigger = leftTrigger;
    this.limelight = limelight;

    this.lockToTagXController = new PIDController(0.01, 0, 0);

    addRequirements(this.drivebase);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    var xy = speedXY.get();
    var r = rot.getAsDouble();

    if (rot.getAsDouble() > DriveConstants.deadband) {
      r = DriveConstants.teleOpNormalAngularSpeed;
    }

    if (lockOnMode && Math.abs(r) < 0.05) {
      r = calculateLockOnRotationSpeed();
    }

    if (leftTrigger > 0.5) {
      xy[0] = xy[0] / 2;
      xy[1] = xy[1] / 2;
      r = DriveConstants.teleOpSlowAngularSpeed;
    }

    drivebase.defaultDrive(-xy[1], -xy[0], r);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public double calculateLockOnRotationSpeed() {
    return lockToTagXController.calculate(limelight.getTx());
  }
}