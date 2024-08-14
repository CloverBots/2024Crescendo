// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
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
import frc.robot.commands.AutoAimAndFireCommand;
import frc.robot.commands.AutoSetShooterCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.Constants.*;
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
    public final VisionTargetTracker visionTargetTracker = new VisionTargetTracker(
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

        autoChooser = AutoBuilder.buildAutoChooser("Test Center");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerveSubsystem.setDefaultCommand(
                new DriveCommand(
                        swerveSubsystem,
                        () -> getScaledXY(),
                        () -> scaleRotationAxis(driverController.getRightX()), driverController.getLeftTriggerAxis(),
                        visionTargetTracker));

        configureBindings();
    }

    // Will run once any time the robot is enabled, in any mode (Doesn't matter if
    // Teleop / Autonomous)
    public void onEnable() {
        resetGyro();
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
        var alliance = DriverStation.getAlliance();
        if (alliance.get() == DriverStation.Alliance.Red) {
            ledSubsystem.conformToState(State.BREATHING_RED);
        } else {
            ledSubsystem.conformToState(State.BREATHING_BLUE);
        }
    }

    public void resetGyro() {
        gyro.reset();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /** Will run once any time the robot is disabled. */
    public void onDisable() {
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
                new AutoIntakeCommand(intakeSubsystem, IntakeConstants.INTAKE_SPEED, feederDistanceSensorSubsystem,
                        feederSubsystem,
                        pivotSubsystem));
        NamedCommands.registerCommand("Fire",
                new AutoAimAndFireCommand(visionTargetTracker, pivotSubsystem, shooterSubsystem, feederSubsystem, 1.0));
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
}
