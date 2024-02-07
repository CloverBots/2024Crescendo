// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Drives the robot to a given X / Y position, and angle. This is relative to the robot's initial position. Positive X is back, Positive Y is Right
 */
public class DriveToDistanceCommand extends Command {
  private SwerveSubsystem swerveSubsystem;

  private PIDController driveDistanceControllerX = new PIDController(7.0, 0.25, 0.1); // p-25, i-0, d-1.5
  private PIDController driveDistanceControllerY = new PIDController(7.0, 0.25, 0.1); // same as above
  private PIDController rotationController = new PIDController(13.0, 0.0, 1.2); // in degrees

  private Timer timer;
  private double timeout;
  /**
   * Drives the robot to a given X / Y position, and angle. This is relative to the robot's initial position.
   * @param swerveSubsystem
   * @param xPos The X position you want the robot to go to. This is forwards/backwards.
   * @param yPos The Y position you want the robot to go to. This is left/right.
   * @param angle The angle, in Radians, that the robot should be at.
   * @param timeout The maximum amount of time, in seconds, that this command should run for. The command will end if the timeout is reached and if it hasn't ended already.
   */
  public DriveToDistanceCommand(SwerveSubsystem swerveSubsystem, double xPos, double yPos, double angle, double timeout) {
    addRequirements(swerveSubsystem);
    this.timer = new Timer();
    this.timeout = timeout;
    this.swerveSubsystem = swerveSubsystem;
    driveDistanceControllerX.setSetpoint(xPos);
    driveDistanceControllerX.setTolerance(0.025); // 0.05 meters = 2 inches
    driveDistanceControllerY.setSetpoint(yPos);
    driveDistanceControllerY.setTolerance(0.025); // 0.05 meters = 2 inches
    rotationController.setSetpoint(angle);
    rotationController.setTolerance(1); // 3 degrees, 0.05 radians
    rotationController.enableContinuousInput(0, 360); // Sets the PID to treat zero and 2 pi as the same value.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Status", true);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = driveDistanceControllerX.calculate(swerveSubsystem.getPose().getX());
    xSpeed = MathUtil.clamp(xSpeed, -SwerveDriveConstants.AUTO_MAX_SPEED, SwerveDriveConstants.AUTO_MAX_SPEED);
  
    double ySpeed = driveDistanceControllerY.calculate(swerveSubsystem.getPose().getY());
    ySpeed = MathUtil.clamp(ySpeed, -SwerveDriveConstants.AUTO_MAX_SPEED, SwerveDriveConstants.AUTO_MAX_SPEED);

    double dTheta = rotationController.calculate(swerveSubsystem.getHeading());
    dTheta = MathUtil.clamp(dTheta, -100, 100);
    
    swerveSubsystem.setSpeed(xSpeed, ySpeed, -dTheta, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
    SmartDashboard.putBoolean("Status", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (driveDistanceControllerX.atSetpoint() && driveDistanceControllerY.atSetpoint() && rotationController.atSetpoint()) 
    || (timer.get() >= timeout);
  
  }
}
