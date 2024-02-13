// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoOneBlue;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveFromControllerCommand;
import frc.robot.commands.FireCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RotateTag;
import frc.robot.commands.ShooterCommand;
import frc.robot.constants.IDs;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotEncoderSubsystem;
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

  private static final double FEEDER_RPM = 20;
  public static final double PIVOT_SPEED = 0.2;

  private static final double CLIMBER_READY_POSITION = 10;
  private static final double CLIMBER_RAISED_POSITION = 15;

  // PARKED SHOOTER
  private static final double SHOOTER_PARKED_RIGHT_RPM = 20;
  private static final double SHOOTER_PARKED_LEFT_RPM = 20;
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
  public static final double AUTO_PIVOT_ANGLE = -999;

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem = new FeederDistanceSensorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private final PivotEncoderSubsystem pivotEncoderSubsystem = new PivotEncoderSubsystem();
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
      driverController::getPOV);

  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, feederSubsystem,
      feederDistanceSensorSubsystem, operatorController::getLeftTriggerAxis, operatorController::getRightTriggerAxis);

  private final ClimbCommand climbReadyCommand = new ClimbCommand(pivotSubsystem, pivotEncoderSubsystem,
      CLIMBER_READY_POSITION);

  private final ClimbCommand climbRaisedCommand = new ClimbCommand(pivotSubsystem, pivotEncoderSubsystem,
      CLIMBER_RAISED_POSITION);

  private final RotateTag rotateTag = new RotateTag(swerveSubsystem, visionTargetTracker);

  private final ShooterCommand parkedShooterCommand = new ShooterCommand(feederDistanceSensorSubsystem,
      shooterSubsystem, pivotEncoderSubsystem, pivotSubsystem, visionTargetTracker,
      SHOOTER_PARKED_LEFT_RPM, SHOOTER_PARKED_RIGHT_RPM,
      false, SHOOTER_PARKED_PIVOT_ANGLE);

  private final ShooterCommand ampShooterCommand = new ShooterCommand(feederDistanceSensorSubsystem,
      shooterSubsystem, pivotEncoderSubsystem, pivotSubsystem, visionTargetTracker,
      SHOOTER_AMP_LEFT_RPM, SHOOTER_AMP_RIGHT_RPM,
      false, SHOOTER_AMP_PIVOT_ANGLE);

  private final ShooterCommand trapShooterCommand = new ShooterCommand(feederDistanceSensorSubsystem,
      shooterSubsystem, pivotEncoderSubsystem, pivotSubsystem, visionTargetTracker,
      SHOOTER_TRAP_LEFT_RPM, SHOOTER_TRAP_RIGHT_RPM, false,
      SHOOTER_TRAP_PIVOT_ANGLE);

  private final ShooterCommand speakerShooterCommand = new ShooterCommand(feederDistanceSensorSubsystem,
      shooterSubsystem, pivotEncoderSubsystem, pivotSubsystem, visionTargetTracker,
      SHOOTER_SPEAKER_LEFT_RPM, SHOOTER_SPEAKER_RIGHT_RPM, true,
      SHOOTER_SPEAKER_PIVOT_ANGLE);

  private final FireCommand fireCommand = new FireCommand(feederSubsystem, shooterSubsystem, pivotEncoderSubsystem,
      pivotSubsystem, FEEDER_RPM);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(driveFromControllerCommand);
    intakeSubsystem.setDefaultCommand(intakeCommand);

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
    swerveSubsystem.setBrakeMode(false);
  }

  public void resetGyro() {
    swerveSubsystem.zeroHeading();
  }

  private void configureAutoChooser() {
    chooser.setDefaultOption("Test Auto", new AutoOneBlue(swerveSubsystem));

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

    JoystickButton fireButton = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    fireButton.onTrue(fireCommand);
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
