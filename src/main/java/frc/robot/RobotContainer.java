// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoAimAndFireCommand;
import frc.robot.commands.AutoSetShooterCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.LEDs.LEDSubsystem.State;
import limelight.LimelightTargetTracking;

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
    public final LimelightTargetTracking visionTargetTracker = new LimelightTargetTracking(
            VisonConstants.visionConfiguration);
    private final AHRS gyro = new AHRS();
    private final Field2d field;
    private final SendableChooser<Command> autoChooser;
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(gyro);
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem = new FeederDistanceSensorSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();

    private final XboxController driverController = new XboxController(Constants.CONTROLLER_DRIVE_PORT);
    private final XboxController operatorController = new XboxController(Constants.CONTROLLER_OPERATOR_PORT);

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

        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerveSubsystem.setDefaultCommand(
                new DriveCommand(
                        swerveSubsystem,
                        () -> getScaledXY(),
                        () -> scaleRotationAxis(driverController.getRightX()), visionTargetTracker));

        configureBindings();
    }

    // Will run once any time the robot is enabled, in any mode (Doesn't matter if
    // Teleop / Autonomous)
    public void onEnable() {
        resetGyro();
    }

    public static Alliance getAlliance() {
        return DriverStation.getAlliance().isEmpty() ? Alliance.Blue : DriverStation.getAlliance().get();
    }

    public void teleopInit() {
        shooterSubsystem.setDefaultCommand(shooterCommand);
    }

    public void teleopPeriodic() {
        feederDistanceSensorSubsystem.isNoteLoaded();
        if (driverController.getStartButton() == true) {
            resetGyro();
        }
    }

    public void onAutonomousEnable() {
        resetGyro();
        ledSubsystem.conformToState(State.RAINBOW);
    }

    public void resetGyro() {
        gyro.reset();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /** Will run once any time the robot is disabled. */
    public void disabledInit() {
        ledSubsystem.conformToState(State.RAINBOW);
    }

    private void configureBindings() {
        // used during tuning
        SmartDashboard.putNumber("Shooter right RPM", 2500);
        SmartDashboard.putNumber("Shooter left RPM", 2000);
        SmartDashboard.putNumber("Shooter angle", 66);
    }

    private void configureAutoCommands() {
        NamedCommands.registerCommand("Intake",
                new AutoIntakeCommand(intakeSubsystem, IntakeConstants.INTAKE_SPEED, feederDistanceSensorSubsystem,
                        feederSubsystem,
                        pivotSubsystem));
        NamedCommands.registerCommand("Fire",
                new AutoAimAndFireCommand(swerveSubsystem, visionTargetTracker, pivotSubsystem, shooterSubsystem,
                        feederSubsystem, 0.5));
        NamedCommands.registerCommand("Set Shooter Subwoofer",
                new AutoSetShooterCommand(shooterSubsystem, pivotSubsystem, feederSubsystem,
                        ShooterConstants.SHOOTER_SPEAKER_LEFT_RPM, ShooterConstants.SHOOTER_SPEAKER_RIGHT_RPM,
                        PivotConstants.PIVOT_SPEAKER_ANGLE));
        NamedCommands.registerCommand("Set Shooter Line",
                new AutoSetShooterCommand(shooterSubsystem, pivotSubsystem, feederSubsystem,
                        ShooterConstants.SHOOTER_LINE_LEFT_RPM, ShooterConstants.SHOOTER_LINE_RIGHT_RPM,
                        PivotConstants.PIVOT_LINE_ANGLE));
        NamedCommands.registerCommand("Set Shooter Far",
                new AutoSetShooterCommand(shooterSubsystem, pivotSubsystem, feederSubsystem,
                        ShooterConstants.SHOOTER_FAR_LEFT_RPM, ShooterConstants.SHOOTER_FAR_RIGHT_RPM,
                        PivotConstants.PIVOT_FAR_ANGLE));

    }

    private double squared(double input) {
        return Math.copySign(input * input, input);
    }

    private double scaleRotationAxis(double input) {
        double rotation = -deadband(squared(input), DriveConstants.deadband) * swerveSubsystem.getMaxAngleVelocity();
        if (driverController.getLeftTriggerAxis() > 0.5) {
            rotation = rotation * Constants.DriveConstants.TELEOP_SLOW_ANGULAR_SCALE_FACTOR;
        } else {
            rotation = rotation * Constants.DriveConstants.TELEOP_NORMAL_ANGULAR_SCALE_FACTOR;
        }
        return rotation;
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

        if (driverController.getLeftTriggerAxis() > 0.5) {
            xy[0] = xy[0] / 25;
            xy[1] = xy[1] / 25;
        }

        return xy;
    }
}
