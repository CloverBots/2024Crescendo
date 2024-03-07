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
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoFarCommand;
import frc.robot.commands.AutoFireCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoLeftCommand;
import frc.robot.commands.AutoLineCommand;
import frc.robot.commands.AutoSubwooferCommand;
import frc.robot.commands.DriveFromControllerCommand;
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
  private static final double VISION_TARGET_HEIGHT = 57.13; // AprilTag 4, 7
  private static final double CAMERA_HEIGHT = 8.75; // inches
  private static final double CAMERA_PITCH = 35; // degrees

  public final static double PIVOT_LOWER_ENDPOINT = 5; // 7
  public final static double PIVOT_UPPER_ENDPOINT = 100;

  private final VisionConfiguration visionConfiguration = new VisionConfiguration(
      VISION_TARGET_HEIGHT,
      CAMERA_HEIGHT,
      CAMERA_PITCH);

  public final VisionTargetTracker visionTargetTracker = new VisionTargetTracker(visionConfiguration);

  public static final double INTAKE_SPEED = 1;
  public static final double FEEDER_SPEED_INTAKE = 0.5;
  public static final double FEEDER_SPEED_SHOOT = 0.8;
  public final static double FEEDER_TIME = 2;
  public static final double PIVOT_SPEED = 0.5;
  public static final double DEFAULT_SPEAKER_PIVOT_ANGLE = 20;

  public static final double CLIMBER_READY_POSITION = 90;
  public static final double CLIMBER_RAISED_POSITION = 15;
  public static final double CLIMBER_PIVOT_SPEED = 0.2;

  // PARKED SHOOTER
  public static final double SHOOTER_PARKED_PIVOT_ANGLE = 10;

  // AMP SHOOTER
  public static final double SHOOTER_AMP_RIGHT_RPM = 600;
  public static final double SHOOTER_AMP_LEFT_RPM = 600;
  public static final double SHOOTER_AMP_PIVOT_ANGLE = 77; // 77

  // TRAP SHOOTER
  public static final double SHOOTER_TRAP_RIGHT_RPM = 20;
  public static final double SHOOTER_TRAP_LEFT_RPM = 20;
  public static final double SHOOTER_TRAP_PIVOT_ANGLE = 50;

  // SPEAKER SHOOTER
  public static final double SHOOTER_SPEAKER_RIGHT_RPM = 2500; // 2500
  public static final double SHOOTER_SPEAKER_LEFT_RPM = 2000; // 2000
  public static final double SHOOTER_SPEAKER_PIVOT_ANGLE = 64; // 64

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
      driverController::getStartButton,
      driverController::getLeftTriggerAxis,
      driverController::getRightTriggerAxis,
      driverController::getPOV,
      visionTargetTracker);

  private final ShooterCommand shooterCommand = new ShooterCommand(feederDistanceSensorSubsystem,
      shooterSubsystem, pivotSubsystem, feederSubsystem, intakeSubsystem, visionTargetTracker,
      operatorController::getRightTriggerAxis,
      operatorController::getLeftTriggerAxis,
      operatorController::getYButton,
      operatorController::getBButton,
      operatorController::getAButton,
      operatorController::getXButton,
      operatorController::getStartButton,
      operatorController::getLeftY,
      operatorController::getPOV,
      operatorController::getBackButton,
      driverController::getRightBumper);

  private final AutoSubwooferCommand autoSubwooferCommand = new AutoSubwooferCommand(feederSubsystem, pivotSubsystem, shooterSubsystem);
  private final AutoLineCommand autoLineCommand = new AutoLineCommand(feederSubsystem, pivotSubsystem, shooterSubsystem);
  private final AutoFarCommand autoFarCommand = new AutoFarCommand(feederSubsystem, pivotSubsystem, shooterSubsystem);
  private final AutoLeftCommand autoLeftCommand = new AutoLeftCommand(feederSubsystem, pivotSubsystem, shooterSubsystem);
  private final AutoFireCommand autoFireCommand = new AutoFireCommand(feederSubsystem, pivotSubsystem, shooterSubsystem);

  private final AutoAimCommand autoAimCommand = new AutoAimCommand(swerveSubsystem, visionTargetTracker, pivotSubsystem, shooterSubsystem, 2.0f);

  private final AutoIntakeCommand autoIntakeCommand = new AutoIntakeCommand(feederDistanceSensorSubsystem,
      feederSubsystem, intakeSubsystem, 5.0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(driveFromControllerCommand);
    shooterSubsystem.setDefaultCommand(shooterCommand);

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      field.getObject("target pose").setPose(pose);
    });
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field.getObject("path").setPoses(poses);
    });

    // Register Named Commands
    NamedCommands.registerCommand("Intake", Commands.print("Intaking")); // autoIntakeCommand
    NamedCommands.registerCommand("Aim", Commands.print("Aiming")); // autoTrackCommand
    NamedCommands.registerCommand("Subwoofer", Commands.print("Subwoofer")); // autoSubwooferCommand
    NamedCommands.registerCommand("Line", Commands.print("Line")); // autoLineCommand
    NamedCommands.registerCommand("Far", Commands.print("Far")); // autoFarCommand
    NamedCommands.registerCommand("Left", Commands.print("Left")); // autoLeftCommand
    NamedCommands.registerCommand("Fire", Commands.print("Fire")); // autoFireCommand

    autoChooser = AutoBuilder.buildAutoChooser("Test X");
    autoChooser.addOption("Test X", AutoBuilder.buildAuto("Test X"));
    autoChooser.addOption("Test Y", AutoBuilder.buildAuto("Test Y"));
    autoChooser.addOption("Test Rotate", AutoBuilder.buildAuto("Test Rotate"));
    autoChooser.addOption("Center Notes 3,2,1,4", AutoBuilder.buildAuto("Center Notes 3,2,1,4"));
    autoChooser.addOption("Center Notes 2,3,6", AutoBuilder.buildAuto("Center Notes 2,3,6"));
    autoChooser.addOption("Right 1,2,3,4", AutoBuilder.buildAuto("Right 1,2,3,4"));
    autoChooser.addOption("Right Notes 4,5,6", AutoBuilder.buildAuto("Right Notes 4,5,6"));
    autoChooser.addOption("Left 3,2,1,4", AutoBuilder.buildAuto("Left 3,2,1,4"));
    autoChooser.addOption("Left Notes 8,7,6", AutoBuilder.buildAuto("Left Notes 8,7,6"));
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
    swerveSubsystem.setBrakeMode(true);
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

    // used during tuning
    SmartDashboard.putNumber("Shooter right RPM", 0.5);
    SmartDashboard.putNumber("Shooter left RPM", 0.5);
    SmartDashboard.putNumber("Shooter angle", 25);
    SmartDashboard.putNumber("Feeder power", 0);

  }
}
