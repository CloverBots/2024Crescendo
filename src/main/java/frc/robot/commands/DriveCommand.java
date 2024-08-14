// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionTargetTracker;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCommand extends Command {

  private final SwerveSubsystem drivebase;
  private VisionTargetTracker limelight;
  private final Supplier<double[]> speedXY;
  private final DoubleSupplier rot;
  private final double leftTrigger;
  public static boolean lockOnMode = false;
  public static boolean slowMode = false;

  private PIDController lockToTagXController;

  /** Creates a new Drive. */
  public DriveCommand(SwerveSubsystem drivebase, Supplier<double[]> speedXY, DoubleSupplier rot, double leftTrigger, VisionTargetTracker limelight) {
    this.drivebase = drivebase;
    this.speedXY = speedXY;
    this.rot = rot;
    this.leftTrigger = leftTrigger;
    this.limelight = limelight;

    this.lockToTagXController = new PIDController(0.075, 0.03, 0.005);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xy = speedXY.get();
    var r = rot.getAsDouble();

    if (lockOnMode && Math.abs(r) < 0.05) {
      r = calculateLockOnRotationSpeed();
    }

    if (leftTrigger >= 0.5 && slowMode == false) {
      slowMode = !slowMode;
    }
    drivebase.defaultDrive(-xy[1], -xy[0], r);
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

  public double calculateLockOnRotationSpeed() {
    return -lockToTagXController.calculate(limelight.getTx());
  }
}