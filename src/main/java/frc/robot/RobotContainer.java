// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

import com.ctre.phoenix6.SignalLogger;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final AHRS gyro = new AHRS();

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(gyro);

  private static XboxController driverController = new XboxController(0);
  private static XboxController operatorController = new XboxController(1);

  private SendableChooser<Command> chooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SignalLogger.enableAutoLogging(false);

    swerveSubsystem.setDefaultCommand(
        new DriveCommand(
            swerveSubsystem,
            () -> getScaledXY(),
            () -> scaleRotationAxis(driverController.getRightX())));

    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", chooser);

    configureAutoChooser();
    SmartDashboard.putData(chooser);
    // Configure the trigger bindings
    configureBindings();
  }

  public void onEnable() {
    resetGyro();
  }

  public void teleopPeriodic() {}

  public void onAutonomousEnable() {}

  private void configureAutoChooser() {}

  public void onDisable() {}

  private void configureBindings() {}

  public void resetGyro() {
    gyro.reset();
  }

  private double squared(double input) {
    return Math.copySign(input * input, input);
  }

  private double scaleRotationAxis(double input) {
    return deadband(squared(input), DriveConstants.deadband) * swerveSubsystem.getMaxAngleVelocity() * -0.6; 
  }

  private double deadband(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0;
    } else {
      return input;
    }
  }

  private double[] getXY() {
    double[] xy = new double[2];
    xy[0] = deadband(driverController.getLeftX(), DriveConstants.deadband);
    xy[1] = deadband(driverController.getLeftY(), DriveConstants.deadband);
    return xy;
  }

  private double[] getScaledXY() {
    double[] xy = getXY();

    // Convert to Polar coordinates
    double r = Math.sqrt(xy[0] * xy[0] + xy[1] * xy[1]);
    double theta = Math.atan2(xy[1], xy[0]);

    // Square radius and scale by max velocity
    r = r * r * swerveSubsystem.getMaxVelocity();

    // Convert to Cartesian coordinates
    xy[0] = r * Math.cos(theta); 
    xy[1] = r * Math.sin(theta); 

    return xy;
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}