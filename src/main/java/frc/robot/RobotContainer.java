// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  private final DriveCommand driveFromControllerCommand = new DriveCommand(
      swerveSubsystem,
      driverController::getLeftX,
      driverController::getLeftY,
      driverController::getRightX,
      driverController::getRightY,
      driverController::getYButton,
      driverController::getBButton,
      driverController::getAButton,
      driverController::getXButton,
      driverController::getStartButton,
      driverController::getLeftTriggerAxis,
      driverController::getRightTriggerAxis,
      driverController::getPOV);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SignalLogger.enableAutoLogging(false);

    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", chooser);

    swerveSubsystem.setDefaultCommand(driveFromControllerCommand);

    configureAutoChooser();
    SmartDashboard.putData(chooser);
    // Configure the trigger bindings
    configureBindings();
  }

  public void onEnable() {}

  public void teleopPeriodic() {}

  public void onAutonomousEnable() {}

  private void configureAutoChooser() {}

  public void onDisable() {}

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}