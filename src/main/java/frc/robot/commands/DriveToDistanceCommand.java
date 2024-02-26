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
 * Drives the robot to a given X / Y position, and angle. This is relative to
 * the robot's initial position. Positive X is back, Positive Y is Right
 */
public class DriveToDistanceCommand extends Command {
  private SwerveSubsystem swerveSubsystem;

  private PIDController driveDistanceControllerX = new PIDController(6.0, 0.5, 0.4); // 6.0, 0.25, 0.4
  private PIDController driveDistanceControllerY = new PIDController(6.0, 0.5, 0.4); //
  private PIDController rotationController = new PIDController(5.0, .3, 0); // 13, .025, 1.2

  double xPos, yPos, angle;
  boolean rotateOnly;
  // SlewRateLimiter xLimiter, yLimiter, rotationLimiter;

  private Timer timer;
  private double timeout;

  /**
   * Drives the robot to a given X / Y position, and angle. This is relative to
   * the robot's initial position.
   * 
   * @param swerveSubsystem
   * @param xPos            The X position you want the robot to go to. This is
   *                        forwards/backwards.
   * @param yPos            The Y position you want the robot to go to. This is
   *                        left/right.
   * @param angle           The angle, in Radians, that the robot should be at.
   * @param timeout         The maximum amount of time, in seconds, that this
   *                        command should run for. The command will end if the
   *                        timeout is reached and if it hasn't ended already.
   */
  public DriveToDistanceCommand(SwerveSubsystem swerveSubsystem, double xPos, double yPos, double angle, double timeout,
      boolean rotateOnly) {
    addRequirements(swerveSubsystem);
    this.timer = new Timer();
    this.timeout = timeout;
    this.swerveSubsystem = swerveSubsystem;

    this.xPos = xPos;
    this.yPos = yPos;
    this.angle = angle;
    this.rotateOnly = rotateOnly;

    driveDistanceControllerX.setSetpoint(xPos);
    driveDistanceControllerX.setTolerance(0.0254); // 0.05 meters = 2 inches
    driveDistanceControllerY.setSetpoint(yPos);
    driveDistanceControllerY.setTolerance(0.0254); // 0.05 meters = 2 inches
    rotationController.setSetpoint(angle);
    rotationController.setTolerance(1.0); // 3 degrees, 0.05 radians
    rotationController.enableContinuousInput(0, 360); // Sets the PID to treat zero and 2 pi as the same value.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    /**
     * Pose2d initPos = swerveSubsystem.getPose();
     * double initAngle = swerveSubsystem.getHeading();
     * if (xPos - initPos.getX() >= 0) xLimiter = new
     * SlewRateLimiter(SwerveDriveConstants.AUTO_MAX_ACCELERATION,
     * Integer.MIN_VALUE, 0);
     * else xLimiter = new SlewRateLimiter(Integer.MAX_VALUE,
     * -SwerveDriveConstants.AUTO_MAX_ACCELERATION, 0);
     * 
     * if (yPos - initPos.getY() >= 0) yLimiter = new
     * SlewRateLimiter(SwerveDriveConstants.AUTO_MAX_ACCELERATION,
     * Integer.MIN_VALUE, 0);
     * else yLimiter = new SlewRateLimiter(Integer.MAX_VALUE,
     * -SwerveDriveConstants.AUTO_MAX_ACCELERATION, 0);
     * 
     * if (angle - initAngle >= 0) rotationLimiter = new
     * SlewRateLimiter(SwerveDriveConstants.AUTO_MAX_ANGULAR_ACCELERATION,
     * Integer.MIN_VALUE, 0);
     * else rotationLimiter = new SlewRateLimiter(Integer.MAX_VALUE,
     * -SwerveDriveConstants.AUTO_MAX_ANGULAR_ACCELERATION, 0);
     **/
    driveDistanceControllerX.reset();
    driveDistanceControllerY.reset();
    rotationController.reset();
    SmartDashboard.putBoolean("Status", true);

    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = driveDistanceControllerX.calculate(swerveSubsystem.getPose().getX());

    // xSpeed = xLimiter.calculate(xSpeed);
    xSpeed = MathUtil.clamp(xSpeed, -SwerveDriveConstants.AUTO_MAX_SPEED, SwerveDriveConstants.AUTO_MAX_SPEED);

    double ySpeed = driveDistanceControllerY.calculate(swerveSubsystem.getPose().getY());
    // ySpeed = yLimiter.calculate(ySpeed);
    ySpeed = MathUtil.clamp(ySpeed, -SwerveDriveConstants.AUTO_MAX_SPEED, SwerveDriveConstants.AUTO_MAX_SPEED);

    double dTheta = rotationController.calculate(swerveSubsystem.getPose().getRotation().getDegrees());
    // dTheta = rotationLimiter.calculate(dTheta);
    // dTheta = MathUtil.clamp(dTheta, -150, 150);

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
    
    if ((timer.get() >= timeout)) return true;

    if ((driveDistanceControllerX.atSetpoint() && driveDistanceControllerY.atSetpoint())) {
      if (rotateOnly) {
        return rotationController.atSetpoint();
      }
      return true;
    } else {
      return false;
    }

  }
}