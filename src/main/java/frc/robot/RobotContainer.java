// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Paths;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveFromControllerCommand;
import frc.robot.constants.IDs;
import frc.robot.constants.PathPlannerConstants;
import frc.robot.constants.PathPlannerConstants.Swerve;
import frc.robot.subsystems.PathSwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static final double VISION_TARGET_HEIGHT = 78.5; // on test robot
  private static final double CAMERA_HEIGHT = 43.7; // on test robot
  private static final double CAMERA_PITCH = 22.0;

  private final VisionConfiguration visionConfiguration = new VisionConfiguration(
      VISION_TARGET_HEIGHT,
      CAMERA_HEIGHT,
      CAMERA_PITCH);

  private final VisionTargetTracker visionTargetTracker = new VisionTargetTracker(visionConfiguration);

  private final Field2d field;
  private final SendableChooser<Command> autoChooser;
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final PathSwerveSubsystem pathSwerveSubsystem = new PathSwerveSubsystem();

  private final XboxController driverController = new XboxController(IDs.CONTROLLER_DRIVE_PORT);
  private final XboxController operatorController = new XboxController(IDs.CONTROLLER_OPERATOR_PORT);

  private final DriveFromControllerCommand driveFromControllerCommand = new DriveFromControllerCommand(
      swerveSubsystem,
      driverController::getLeftX,
      driverController::getLeftY,
      driverController::getRightX,
      driverController::getRightY,
      driverController::getYButton,
      driverController::getBButton,
      driverController::getAButton,
      driverController::getXButton,
      driverController::getLeftTriggerAxis,
      driverController::getRightTriggerAxis,
      driverController::getPOV,
      visionTargetTracker);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(driveFromControllerCommand);

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      field.getObject("target pose").setPose(pose);
    });
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field.getObject("path").setPoses(poses);
    });

    // Register Named Commands
    /* NamedCommands.registerCommand("Intake", intakeCommandAuto);
    NamedCommands.registerCommand("Start Shooter Center", autoShootCommandCenter);
    NamedCommands.registerCommand("Start Shooter Side", autoShootCommandSide);
    NamedCommands.registerCommand("Fire", fireCommand); */

    autoChooser = AutoBuilder.buildAutoChooser("Test");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  /**
   * Will run once any time the robot is enabled, in any mode (Doesn't matter if
   * Teleop / Autonomous)
   */
  public void onEnable() {
    swerveSubsystem.onEnable();
    swerveSubsystem.setBrakeMode(true);
  }

  public void teleopPeriodic() {
  }

  public void onAutonomousEnable() {
    swerveSubsystem.setBrakeMode(false);
  }

  public void resetGyro() {
    swerveSubsystem.zeroHeading();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /** Will run once any time the robot is disabled. */
  public void onDisable() {
    swerveSubsystem.setBrakeMode(false);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  }
}
