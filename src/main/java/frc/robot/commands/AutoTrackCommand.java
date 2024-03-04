// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionTargetTracker;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoTrackCommand extends Command {

  private PIDController lockToTagXController = new PIDController(0.075, 0.03, 0.005);
  private SwerveSubsystem swerve;
  private VisionTargetTracker visionTargetTracker;
  private double pivotAngle;
  private double previousPivotAngle;
  private float time;
  private Timer timer;

  public AutoTrackCommand(SwerveSubsystem swerve, VisionTargetTracker visionTargetTracker,
      float time) {
    addRequirements(swerve);
    this.swerve = swerve;
    this.lockToTagXController = new PIDController(14.0, 0.7, 0.3);
    this.lockToTagXController.setTolerance(2);
    this.visionTargetTracker = visionTargetTracker;
    this.time = time;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lockToTagXController.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double calculate = -lockToTagXController.calculate(visionTargetTracker.getTx(), 0);
    swerve.setSpeed(0, 0, calculate, true);

    SmartDashboard.putNumber("calculate", calculate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!visionTargetTracker.isValid() || timer.get() > time) {
      return true;
    }

    return lockToTagXController.atSetpoint();
  }
}