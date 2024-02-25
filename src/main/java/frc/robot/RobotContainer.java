// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoOneBlue;
import frc.robot.commands.AutoTwoBlue;
import frc.robot.commands.DriveFromControllerCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.constants.IDs;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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

  public final static double PIVOT_LOWER_ENDPOINT = 7; // 7
  public final static double PIVOT_UPPER_ENDPOINT = 100;

  private final VisionConfiguration visionConfiguration = new VisionConfiguration(
      VISION_TARGET_HEIGHT,
      CAMERA_HEIGHT,
      CAMERA_PITCH);

  public final VisionTargetTracker visionTargetTracker = new VisionTargetTracker(visionConfiguration);

  public static final double INTAKE_SPEED = 1;
  public static final double FEEDER_SPEED_INTAKE = 0.6;
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
  public static final double SHOOTER_SPEAKER_RIGHT_RPM = 4500;
  public static final double SHOOTER_SPEAKER_LEFT_RPM = 3500;
  public static final double SHOOTER_SPEAKER_PIVOT_ANGLE = 31; // Automatic 55 at 79in, 36 at 212in, 39 at 144in

  // Used to indicate auto mode (based on April tag distance) for the shooter
  // pivot angle
  public static final double AUTO_PIVOT_ANGLE = 25;

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem = new FeederDistanceSensorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final XboxController driverController = new XboxController(IDs.CONTROLLER_DRIVE_PORT);
  private final XboxController operatorController = new XboxController(IDs.CONTROLLER_OPERATOR_PORT);
  private final SendableChooser<Command> chooser = new SendableChooser<>();

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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(driveFromControllerCommand);
    shooterSubsystem.setDefaultCommand(shooterCommand);

    configureAutoChooser();
    SmartDashboard.putData(chooser);

    // Configure the trigger bindings
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

  private void configureAutoChooser() {
    chooser.addOption("Auto One", new AutoOneBlue(swerveSubsystem, feederSubsystem, pivotSubsystem,
        shooterSubsystem, feederDistanceSensorSubsystem,
        intakeSubsystem, visionTargetTracker));
    chooser.setDefaultOption("Auto Two", new AutoTwoBlue(swerveSubsystem, feederSubsystem, pivotSubsystem,
        shooterSubsystem, feederDistanceSensorSubsystem,
        intakeSubsystem, visionTargetTracker));

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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
