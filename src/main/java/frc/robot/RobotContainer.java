// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoFireCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoShooterCommand;
import frc.robot.commands.DriveFromControllerCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.AutoSubwooferCommand;
import frc.robot.commands.autoYEETCommand;
import frc.robot.constants.IDs;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LEDSubsystem.State;

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

  public final static double PIVOT_LOWER_ENDPOINT = 6; // 7 , 5
  public final static double PIVOT_UPPER_ENDPOINT = 100;

  private final VisionConfiguration visionConfiguration = new VisionConfiguration(
      VISION_TARGET_HEIGHT,
      CAMERA_HEIGHT,
      CAMERA_PITCH);

  public final VisionTargetTracker visionTargetTracker = new VisionTargetTracker(visionConfiguration);

  public static final double INTAKE_SPEED = 1;
  public static final double FEEDER_SPEED_INTAKE = 0.5;
  public static final double FEEDER_SPEED_SHOOT = 0.8;
  public final static double FEEDER_TIME = 1;

  public static final double CLIMBER_PIVOT_SPEED = 0.5;

  // PARKED SHOOTER
  public static final double SHOOTER_PARKED_PIVOT_ANGLE = 10;

  // AMP SHOOTER
  public static final double SHOOTER_AMP_RIGHT_RPM = 700;
  public static final double SHOOTER_AMP_LEFT_RPM = 700;
  public static final double SHOOTER_AMP_PIVOT_ANGLE = 70; // 72

  // TRAP SHOOTER
  public static final double SHOOTER_TRAP_RIGHT_RPM = 2000;
  public static final double SHOOTER_TRAP_LEFT_RPM = 2000;
  public static final double SHOOTER_TRAP_PIVOT_ANGLE = 60;

  // SPEAKER SHOOTER
  public static final double SHOOTER_SPEAKER_RIGHT_RPM = 2500; // 2500
  public static final double SHOOTER_SPEAKER_LEFT_RPM = 2000; // 2000
  public static final double SHOOTER_SPEAKER_PIVOT_ANGLE = 66; // 66

  // OVER STAGE
  public static final double SHOOTER_OVER_STAGE_RIGHT_RPM = 3000;
  public static final double SHOOTER_OVER_STAGE_LEFT_RPM = 2500;
  public static final double SHOOTER_OVER_STAGE_PIVOT_ANGLE = 60;

  // UNDER STAGE
  public static final double SHOOTER_UNDER_STAGE_RIGHT_RPM = 3000;
  public static final double SHOOTER_UNDER_STAGE_LEFT_RPM = 3000;
  public static final double SHOOTER_UNDER_STAGE_PIVOT_ANGLE = 10;

  private final Field2d field;
  private final SendableChooser<Command> autoChooser;
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem = new FeederDistanceSensorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();

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
      driverController::getRightBumper,
      operatorController::getLeftBumper);

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
    configureAutoCommands();
    autoChooser = AutoBuilder.buildAutoChooser("Test Center");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  // Will run once any time the robot is enabled, in any mode (Doesn't matter if
  // Teleop / Autonomous)
  public void onEnable() {
    swerveSubsystem.onEnable();
    swerveSubsystem.setBrakeMode(true);
  }

  public void teleopPeriodic() {
    feederDistanceSensorSubsystem.isNoteLoaded();
  }

  public void onAutonomousEnable() {
    swerveSubsystem.setBrakeMode(true);
    resetGyro();
    var alliance = DriverStation.getAlliance();
    if (alliance.get() == DriverStation.Alliance.Red) {
      ledSubsystem.conformToState(State.BREATHING_RED);
    } else {
      ledSubsystem.conformToState(State.BREATHING_BLUE);
    }
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
    ledSubsystem.conformToState(State.RAINBOW);
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
    SmartDashboard.putNumber("Shooter right RPM", 2500);
    SmartDashboard.putNumber("Shooter left RPM", 2000);
    SmartDashboard.putNumber("Shooter angle", 66);

  }

  private void configureAutoCommands() {
    NamedCommands.registerCommand("Intake",
        new AutoIntakeCommand(feederDistanceSensorSubsystem, feederSubsystem, intakeSubsystem));
    NamedCommands.registerCommand("Aim",
        new AutoAimCommand(visionTargetTracker, pivotSubsystem, shooterSubsystem, 2.0f));
    NamedCommands.registerCommand("Subwoofer", new AutoSubwooferCommand(intakeSubsystem, pivotSubsystem, shooterSubsystem, feederSubsystem));
    NamedCommands.registerCommand("Line", new AutoShooterCommand(pivotSubsystem, shooterSubsystem,
        46.5, 2500, 3000)); // TO-DO tune
    NamedCommands.registerCommand("Far", new AutoShooterCommand(pivotSubsystem, shooterSubsystem,
        27, 4000, 4500));
    NamedCommands.registerCommand("Stage", new AutoShooterCommand(pivotSubsystem, shooterSubsystem,
        30, 3000, 4000));
    NamedCommands.registerCommand("Podium", new AutoShooterCommand(pivotSubsystem, shooterSubsystem,
        42, 3000, 3500));
    NamedCommands.registerCommand("Fire", new AutoFireCommand(feederSubsystem, pivotSubsystem, shooterSubsystem));
    NamedCommands.registerCommand("YEET",
        new autoYEETCommand(intakeSubsystem, pivotSubsystem, shooterSubsystem, feederSubsystem));
  }
}
