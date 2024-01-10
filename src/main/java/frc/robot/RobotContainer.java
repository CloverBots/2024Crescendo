// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.commands.AutoToteCrashLeft;
import frc.robot.commands.AutoToteCrashRight;
import frc.robot.commands.AutoNothing;
import frc.robot.commands.AutoRabbit;
import frc.robot.commands.BallIntakeCommand;
import frc.robot.commands.BallShooterCommand;
import frc.robot.commands.DriveFromControllerCommand;
import frc.robot.commands.RabbitDeployCommand;
import frc.robot.commands.RabbitIntakeCommand;
import frc.robot.constants.IDs;
import frc.robot.subsystems.BallDeploySubsystem;
import frc.robot.subsystems.BallIntakeSubsystem;
import frc.robot.subsystems.BallShooterSubsystem;
import frc.robot.subsystems.RabbitDeploySubsystem;
import frc.robot.subsystems.RabbitIntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.BallDeployCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  
  private final XboxController driverController = new XboxController(IDs.CONTROLLER_DRIVE_PORT);
  private final XboxController operatorController = new XboxController(IDs.CONTROLLER_OPERATOR_PORT);
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private final DriveFromControllerCommand driveFromControllerCommand = 
    new DriveFromControllerCommand(
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

  private final RabbitIntakeSubsystem rabbitIntakeSubsystem = new RabbitIntakeSubsystem();
  private final RabbitDeploySubsystem rabbitDeploySubsystem = new RabbitDeploySubsystem();
  // private final BallIntakeSubsystem ballIntakeSubsystem = new BallIntakeSubsystem();
  // private final BallDeploySubsystem ballDeploySubsystem = new BallDeploySubsystem();
  // private final BallShooterSubsystem ballShooterSubsystem = new BallShooterSubsystem();

  private final RabbitIntakeCommand rabbitIntakeCommand = new RabbitIntakeCommand(rabbitIntakeSubsystem, operatorController::getLeftY);
  // private final BallIntakeCommand ballIntakeCommand = new BallIntakeCommand(ballIntakeSubsystem, operatorController::getLeftTriggerAxis, operatorController::getRightTriggerAxis);
  // TO-DO find correct position and speed
  // private final BallDeployCommand BallDeployUpCommand = new BallDeployCommand(ballDeploySubsystem, 15, 0.1);
  // private final BallDeployCommand BallDeployDownCommand = new BallDeployCommand(ballDeploySubsystem, 5, 0.1);
  private final RabbitDeployCommand RabbitDeployGroundCommand = new RabbitDeployCommand(rabbitDeploySubsystem, -34.0);
  private final RabbitDeployCommand RabbitDeployBinCommand = new RabbitDeployCommand(rabbitDeploySubsystem, -18.1); // -18.16
  private final RabbitDeployCommand RabbitDeployUpCommand = new RabbitDeployCommand(rabbitDeploySubsystem, 0);
  // private final BallShooterCommand ballShooterCommand = new BallShooterCommand(ballShooterSubsystem, 50);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(driveFromControllerCommand);
    rabbitIntakeSubsystem.setDefaultCommand(rabbitIntakeCommand);
    // ballIntakeSubsystem.setDefaultCommand(ballIntakeCommand);
    configureAutoChooser();
    SmartDashboard.putData(chooser);
    // Configure the trigger bindings
    configureBindings();
  }

  /** Will run once any time the robot is enabled, in any mode (Doesn't matter if Teleop / Autonomous) */
  public void onEnable() {
    swerveSubsystem.onEnable();
    swerveSubsystem.setBrakeMode(true);
  }
  public void teleopPeriodic() {
  }
  public void resetGyro() {
    swerveSubsystem.zeroHeading();
  }
  private void configureAutoChooser() {
    chooser.setDefaultOption("Get Rabbit", new AutoRabbit(swerveSubsystem, rabbitDeploySubsystem, rabbitIntakeSubsystem));
    chooser.addOption("Nothing", new AutoNothing(swerveSubsystem));
    chooser.addOption("Tote Crash Left", new AutoToteCrashLeft(swerveSubsystem));
    chooser.addOption("Tote Crash Right", new AutoToteCrashRight(swerveSubsystem));
  }
  /** Will run once any time the robot is disabled. */
  public void onDisable() {
    swerveSubsystem.setBrakeMode(false);
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // JoystickButton ballDeployUpButton = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    // JoystickButton ballDeployDownButton = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    JoystickButton rabbitDeployGroundButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
    JoystickButton rabbitDeployBinButton = new JoystickButton(operatorController, XboxController.Button.kB.value);
    JoystickButton rabbitDeployUpButton = new JoystickButton(operatorController, XboxController.Button.kY.value);
    // POVButton dPadUpButton = new POVButton(operatorController, 0);
    
    // ballDeployUpButton.onTrue(BallDeployUpCommand);
    // ballDeployDownButton.onTrue(BallDeployDownCommand);
    rabbitDeployGroundButton.onTrue(RabbitDeployGroundCommand);
    rabbitDeployBinButton.onTrue(RabbitDeployBinCommand);
    rabbitDeployUpButton.onTrue(RabbitDeployUpCommand);
    // dPadUpButton.onTrue(ballShooterCommand);
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
