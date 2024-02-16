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
import frc.robot.commands.AutoOneBlue;
import frc.robot.commands.AutoShooterCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveFromControllerCommand;
import frc.robot.commands.FireCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommandAuto;
import frc.robot.commands.ShooterCommand;
import frc.robot.constants.IDs;
import frc.robot.constants.PathPlannerConstants;
import frc.robot.constants.PathPlannerConstants.Swerve;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PathSwerveSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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

  public static final double FEEDER_SPEED = 0.2;
  public static final double PIVOT_SPEED = 0.2;
  public static final double DEFAULT_SPEAKER_PIVOT_ANGLE = 20;

  private static final double CLIMBER_READY_POSITION = 10;
  private static final double CLIMBER_RAISED_POSITION = 15;

  // PARKED SHOOTER
  private static final double SHOOTER_PARKED_RIGHT_RPM = 0;
  private static final double SHOOTER_PARKED_LEFT_RPM = 0;
  public static final double SHOOTER_PARKED_PIVOT_ANGLE = 0;

  // AMP SHOOTER
  private static final double SHOOTER_AMP_RIGHT_RPM = 20;
  private static final double SHOOTER_AMP_LEFT_RPM = 20;
  private static final double SHOOTER_AMP_PIVOT_ANGLE = 45;

  // TRAP SHOOTER
  private static final double SHOOTER_TRAP_RIGHT_RPM = 20;
  private static final double SHOOTER_TRAP_LEFT_RPM = 20;
  private static final double SHOOTER_TRAP_PIVOT_ANGLE = 45;

  // SPEAKER SHOOTER
  private static final double SHOOTER_SPEAKER_RIGHT_RPM = 20;
  private static final double SHOOTER_SPEAKER_LEFT_RPM = 20;
  private static final double SHOOTER_SPEAKER_PIVOT_ANGLE = -999; // Automatic

  // Used to indicate auto mode (based on April tag distance) for the shooter
  // pivot angle
  public static final double AUTO_PIVOT_ANGLE_SIDE = 0;
  public static final double AUTO_PIVOT_ANGLE = 0;

  private final Field2d field;
  private final SendableChooser<Command> autoChooser;
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final PathSwerveSubsystem pathSwerveSubsystem = new PathSwerveSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem = new FeederDistanceSensorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

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

  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, feederSubsystem,
      feederDistanceSensorSubsystem, operatorController::getLeftTriggerAxis, operatorController::getRightTriggerAxis);

  private final ClimbCommand climbReadyCommand = new ClimbCommand(pivotSubsystem,
      CLIMBER_READY_POSITION);

  private final ClimbCommand climbRaisedCommand = new ClimbCommand(pivotSubsystem,
      CLIMBER_RAISED_POSITION);

  private final ShooterCommand parkedShooterCommand = new ShooterCommand(feederDistanceSensorSubsystem,
      shooterSubsystem, pivotSubsystem, feederSubsystem, visionTargetTracker, driverController::getRightBumper,
      SHOOTER_PARKED_LEFT_RPM, SHOOTER_PARKED_RIGHT_RPM, FEEDER_SPEED,
      false, SHOOTER_PARKED_PIVOT_ANGLE);

  private final ShooterCommand ampShooterCommand = new ShooterCommand(feederDistanceSensorSubsystem,
      shooterSubsystem, pivotSubsystem, feederSubsystem, visionTargetTracker, driverController::getRightBumper,
      SHOOTER_AMP_LEFT_RPM, SHOOTER_AMP_RIGHT_RPM, FEEDER_SPEED,
      false, SHOOTER_AMP_PIVOT_ANGLE);

  private final ShooterCommand trapShooterCommand = new ShooterCommand(feederDistanceSensorSubsystem,
      shooterSubsystem, pivotSubsystem, feederSubsystem, visionTargetTracker, driverController::getRightBumper,
      SHOOTER_TRAP_LEFT_RPM, SHOOTER_TRAP_RIGHT_RPM, FEEDER_SPEED, false,
      SHOOTER_TRAP_PIVOT_ANGLE);

  private final ShooterCommand speakerShooterCommand = new ShooterCommand(feederDistanceSensorSubsystem,
      shooterSubsystem, pivotSubsystem, feederSubsystem, visionTargetTracker, driverController::getRightBumper,
      SHOOTER_SPEAKER_LEFT_RPM, SHOOTER_SPEAKER_RIGHT_RPM, FEEDER_SPEED, true,
      SHOOTER_SPEAKER_PIVOT_ANGLE);

  private final ShooterCommand tuningShooterCommand = new ShooterCommand(feederDistanceSensorSubsystem,
      shooterSubsystem, pivotSubsystem, feederSubsystem, visionTargetTracker, driverController::getRightBumper,
      -1, 0, 0, false,
      0);

  private final FireCommand fireCommand = new FireCommand(feederSubsystem, FEEDER_SPEED);

  private final IntakeCommandAuto intakeCommandAuto = new IntakeCommandAuto(intakeSubsystem, feederSubsystem,
      feederDistanceSensorSubsystem);

  private final AutoShooterCommand autoShootCommandCenter = new AutoShooterCommand(feederDistanceSensorSubsystem,
      shooterSubsystem, pivotSubsystem, fireCommand, SHOOTER_SPEAKER_LEFT_RPM, SHOOTER_SPEAKER_RIGHT_RPM,
      AUTO_PIVOT_ANGLE);

  private final AutoShooterCommand autoShootCommandSide = new AutoShooterCommand(feederDistanceSensorSubsystem,
      shooterSubsystem, pivotSubsystem, fireCommand, SHOOTER_SPEAKER_LEFT_RPM, SHOOTER_SPEAKER_RIGHT_RPM,
      AUTO_PIVOT_ANGLE_SIDE);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(driveFromControllerCommand);
    intakeSubsystem.setDefaultCommand(intakeCommand);

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      field.getObject("target pose").setPose(pose);
    });
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field.getObject("path").setPoses(poses);
    });

    // Register Named Commands
    NamedCommands.registerCommand("Intake", intakeCommandAuto);
    NamedCommands.registerCommand("Start Shooter Center", autoShootCommandCenter);
    NamedCommands.registerCommand("Start Shooter Side", autoShootCommandSide);
    NamedCommands.registerCommand("Fire", fireCommand);

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
    feederDistanceSensorSubsystem.isNoteLoaded();
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
    // JoystickButton followTag = new JoystickButton(operatorController,
    // XboxController.Button.kA.value);
    // followTag.onTrue(rotateTag);

    POVButton climbReadyButton = new POVButton(operatorController, 0); // Up
    climbReadyButton.onTrue(climbReadyCommand);
    POVButton climbRaisedButton = new POVButton(operatorController, 180); // Down
    climbRaisedButton.onTrue(climbRaisedCommand);

    JoystickButton parkedButton = new JoystickButton(operatorController, XboxController.Button.kX.value);
    parkedButton.onTrue(parkedShooterCommand);
    JoystickButton ampButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
    ampButton.onTrue(ampShooterCommand);
    JoystickButton speakerButton = new JoystickButton(operatorController, XboxController.Button.kB.value);
    speakerButton.onTrue(speakerShooterCommand);
    JoystickButton trapButton = new JoystickButton(operatorController, XboxController.Button.kY.value);
    trapButton.onTrue(trapShooterCommand);

    JoystickButton tuningButton = new JoystickButton(operatorController, XboxController.Button.kStart.value);
    tuningButton.onTrue(tuningShooterCommand);
  }
}
