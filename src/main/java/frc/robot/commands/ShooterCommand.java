package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionTargetTracker;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
    private final static double MAX_PIVOT_POWER = 0.2;

    private final ShooterSubsystem shooterSubsystem;
    private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem;
    private final PivotSubsystem pivotSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final VisionTargetTracker visionTargetTracker;
    private final Supplier<Boolean> fireButton;
    private final Supplier<Double> intakeLoadTrigger, intakeEjectTrigger;
    private final DoubleSupplier leftJoystickY;
    private final Supplier<Boolean> yButton, bButton, aButton, xButton, startButton, backButton;
    private final Supplier<Integer> dPad;
    private double shooterLeftRPM;
    private double shooterRightRPM;
    private double feederSpeed;
    private double pivotAngle;
    private double previousPivotAngle;
    private double pivotSpeed = 0;
    private Timer timer;
    private boolean firing = false;

    public enum ACTION {
        NONE, INTAKE, EJECT, AMP, SPEAKER, TRAP, FIRE, PARK, TUNING, CLIMB_AUTO_READY, CLIMB_AUTO_LIFT, CLIMB_MANUAL
    }

    private ACTION mode;
    private boolean modeChanged = false;

    private PIDController pivotController = new PIDController(0.025, 0.0, 0); // in degrees

    public ShooterCommand(FeederDistanceSensorSubsystem feederDistanceSensorSubsystem,
            ShooterSubsystem shooterSubsystem,
            PivotSubsystem pivotSubsystem,
            FeederSubsystem feederSubsystem,
            IntakeSubsystem intakeSubsystem,
            VisionTargetTracker visionTargetTracker,
            Supplier<Double> intakeLoadTrigger,
            Supplier<Double> intakeEjectTrigger,
            Supplier<Boolean> yButton,
            Supplier<Boolean> bButton,
            Supplier<Boolean> aButton,
            Supplier<Boolean> xButton,
            Supplier<Boolean> startButton,
            DoubleSupplier leftJoystickY,
            Supplier<Integer> dPad,
            Supplier<Boolean> backButton,
            Supplier<Boolean> fireButton) {

        this.shooterSubsystem = shooterSubsystem;
        this.feederDistanceSensorSubsystem = feederDistanceSensorSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.visionTargetTracker = visionTargetTracker;
        this.intakeLoadTrigger = intakeLoadTrigger;
        this.intakeEjectTrigger = intakeEjectTrigger;
        this.yButton = yButton;
        this.bButton = bButton;
        this.aButton = aButton;
        this.xButton = xButton;
        this.startButton = startButton;
        this.leftJoystickY = leftJoystickY;
        this.backButton = backButton;
        this.dPad = dPad;
        this.fireButton = fireButton;

        timer = new Timer();
        mode = ACTION.NONE;

        pivotController.setTolerance(1); // 3 degrees, 0.05 radians
        pivotController.enableContinuousInput(0, 360); // Sets the PID to treat zero and 2 pi as the same value.

        addRequirements(shooterSubsystem);
        addRequirements(feederDistanceSensorSubsystem);
        addRequirements(pivotSubsystem);
        addRequirements(feederSubsystem);
        addRequirements(intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        firing = false;
        pivotController.reset();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        checkControllerValues();

        // If mode changed then get appropriate values for the new mode
        if (modeChanged) {
            updateValues();
        }

        switch (mode) {
            case INTAKE:
                if (!feederDistanceSensorSubsystem.isNoteLoaded()) {
                    intakeSubsystem.setIntakeSpeed(RobotContainer.INTAKE_SPEED);
                    feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED);
                    pivotSpeed = -pivotController.calculate(pivotSubsystem.getPivotAbsolutePosition());
                    pivotSpeed = checkPivotSpeed(pivotSpeed);
                    pivotSpeed = MathUtil.clamp(pivotSpeed, -MAX_PIVOT_POWER, MAX_PIVOT_POWER);
                    pivotSubsystem.setSpeed(pivotSpeed);
                }
                break;
            case EJECT:
                intakeSubsystem.setIntakeSpeed(-RobotContainer.INTAKE_SPEED);
                feederSubsystem.setSpeed(-RobotContainer.FEEDER_SPEED);
                pivotSpeed = -pivotController.calculate(pivotSubsystem.getPivotAbsolutePosition());
                pivotSpeed = checkPivotSpeed(pivotSpeed);
                pivotSpeed = MathUtil.clamp(pivotSpeed, -MAX_PIVOT_POWER, MAX_PIVOT_POWER);

                pivotSubsystem.setSpeed(pivotSpeed);
                break;

            case TUNING:
                shooterRightRPM = SmartDashboard.getNumber("Shooter right power", 0);
                shooterLeftRPM = SmartDashboard.getNumber("Shooter left power", 0);
                pivotAngle = SmartDashboard.getNumber("Shooter angle", 25);
                feederSpeed = SmartDashboard.getNumber("Feeder power", feederSpeed);
                feederSubsystem.setSpeed(feederSpeed);
            case PARK:
            case AMP:
            case TRAP:
                shooterSubsystem.setShooterLeftRPM(shooterLeftRPM);
                shooterSubsystem.setShooterRightRPM(shooterRightRPM);

                pivotSpeed = -pivotController.calculate(pivotSubsystem.getPivotAbsolutePosition());
                pivotSpeed = checkPivotSpeed(pivotSpeed);
                pivotSpeed = MathUtil.clamp(pivotSpeed, -MAX_PIVOT_POWER, MAX_PIVOT_POWER);

                pivotSubsystem.setSpeed(pivotSpeed);
                break;

            case SPEAKER:
                shooterSubsystem.setShooterLeftRPM(shooterLeftRPM);
                shooterSubsystem.setShooterRightRPM(shooterRightRPM);

                double targetDistance = visionTargetTracker.computeTargetDistance();
                Boolean isTargetValid = visionTargetTracker.isValid();
                if (isTargetValid) {
                    pivotAngle = computePivotAngle(targetDistance);
                    pivotAngle = checkAngleLimits(pivotAngle);
                }

                // If the desired angle has changed by 1 degree or more, update the setpoint
                if (Math.abs(previousPivotAngle - pivotAngle) > 1) {
                    previousPivotAngle = pivotAngle;
                    pivotController.reset();
                    pivotController.setSetpoint(pivotAngle);
                }

                pivotSpeed = -pivotController.calculate(pivotSubsystem.getPivotAbsolutePosition());
                pivotSpeed = checkPivotSpeed(pivotSpeed);
                pivotSpeed = MathUtil.clamp(pivotSpeed, -MAX_PIVOT_POWER, MAX_PIVOT_POWER);

                pivotSubsystem.setSpeed(pivotSpeed);
                break;
            case FIRE:
                if (!firing) {
                    firing = true;
                    timer.reset();
                    timer.start();
                    feederSubsystem.setSpeed(feederSpeed);
                }
                break;

            case NONE:
                break;

            case CLIMB_AUTO_LIFT:
            case CLIMB_AUTO_READY:
                pivotSubsystem.setSpeed(pivotSpeed);
                break;

            case CLIMB_MANUAL:
                if (Math.abs(leftJoystickY.getAsDouble()) > .05) {
                    pivotSpeed = leftJoystickY.getAsDouble() / 3;

                    if (leftJoystickY.getAsDouble() < -.05 &&
                            pivotSubsystem.getPivotAbsolutePosition() > RobotContainer.PIVOT_UPPER_ENDPOINT) {
                        pivotSpeed = 0;
                    }

                    if (leftJoystickY.getAsDouble() > .05 &&
                            pivotSubsystem.getPivotAbsolutePosition() < RobotContainer.PIVOT_LOWER_ENDPOINT) {
                        pivotSpeed = 0;
                    }

                    pivotSubsystem.setSpeed(pivotSpeed);

                } else {
                    pivotSubsystem.setSpeed(0);
                }

                break;
        }

    }

    private void checkControllerValues() {
        if (intakeLoadTrigger.get() > 0.5 && mode != ACTION.INTAKE) {
            mode = ACTION.INTAKE;
            modeChanged = true;
        } else if (intakeEjectTrigger.get() > 0.5 && mode != ACTION.EJECT) {
            mode = ACTION.EJECT;
            modeChanged = true;
        } else if (xButton.get() && mode != ACTION.PARK) {
            mode = ACTION.PARK;
            modeChanged = true;
        } else if (aButton.get() && mode != ACTION.AMP) {
            mode = ACTION.AMP;
            modeChanged = true;
        } else if (bButton.get() && mode != ACTION.SPEAKER) {
            mode = ACTION.SPEAKER;
            modeChanged = true;
        } else if (yButton.get() && mode != ACTION.TRAP) {
            mode = ACTION.TRAP;
            modeChanged = true;
        } else if (startButton.get() && mode != ACTION.TUNING) {
            mode = ACTION.TUNING;
            modeChanged = true;
        } else if (fireButton.get()) {
            mode = ACTION.FIRE;
            modeChanged = true;
        } else if (backButton.get()) {
            mode = ACTION.CLIMB_MANUAL;
            modeChanged = true;
        } else if (dPad.get() == 0) {
            mode = ACTION.CLIMB_AUTO_READY;
            modeChanged = true;
        } else if (dPad.get() == 180) {
            mode = ACTION.CLIMB_AUTO_LIFT;
            modeChanged = true;
        }
    }

    private void updateValues() {
        modeChanged = false;

        switch (mode) {
            case INTAKE:
            case EJECT:
                shooterSubsystem.setShooterLeftRPM(0);
                shooterSubsystem.setShooterRightRPM(0);
                feederSubsystem.setSpeed(0);
                pivotAngle = RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotController.reset();
                pivotController.setSetpoint(pivotAngle);

                break;

            case PARK:
                shooterLeftRPM = 0;
                shooterRightRPM = 0;
                shooterSubsystem.stop();
                feederSpeed = 0;
                pivotAngle = RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotController.reset();
                pivotController.setSetpoint(pivotAngle);
                DriveFromControllerCommand.lockOnMode = false;
                break;

            case AMP:
                shooterLeftRPM = RobotContainer.SHOOTER_AMP_LEFT_RPM;
                shooterRightRPM = RobotContainer.SHOOTER_AMP_RIGHT_RPM;
                feederSpeed = RobotContainer.FEEDER_SPEED;
                pivotAngle = RobotContainer.SHOOTER_AMP_PIVOT_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotController.reset();
                pivotController.setSetpoint(pivotAngle);
                break;

            case SPEAKER:
                shooterLeftRPM = RobotContainer.SHOOTER_SPEAKER_LEFT_RPM;
                shooterRightRPM = RobotContainer.SHOOTER_SPEAKER_RIGHT_RPM;
                feederSpeed = RobotContainer.FEEDER_SPEED;
                pivotAngle = RobotContainer.SHOOTER_SPEAKER_PIVOT_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                previousPivotAngle = pivotAngle;
                pivotController.reset();
                pivotController.setSetpoint(pivotAngle);

                DriveFromControllerCommand.lockOnMode = true; 
                previousPivotAngle = pivotAngle;

                break;
            case TRAP:
                shooterLeftRPM = RobotContainer.SHOOTER_TRAP_LEFT_RPM;
                shooterRightRPM = RobotContainer.SHOOTER_TRAP_RIGHT_RPM;
                feederSpeed = RobotContainer.FEEDER_SPEED;
                pivotAngle = RobotContainer.SHOOTER_TRAP_PIVOT_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotController.reset();
                pivotController.setSetpoint(pivotAngle);
                break;

            case FIRE:
                feederSpeed = RobotContainer.FEEDER_SPEED;

            case NONE:

                break;

            case CLIMB_AUTO_LIFT:
                shooterLeftRPM = 0;
                shooterRightRPM = 0;
                feederSpeed = 0;
                pivotController.reset();
                pivotAngle = RobotContainer.CLIMBER_RAISED_POSITION;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotSpeed = RobotContainer.CLIMBER_PIVOT_SPEED;
                DriveFromControllerCommand.lockOnMode = false;
                break;

            case CLIMB_AUTO_READY:
                shooterLeftRPM = 0;
                shooterRightRPM = 0;
                feederSpeed = 0;
                pivotController.reset();
                pivotAngle = RobotContainer.CLIMBER_READY_POSITION;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotSpeed = -RobotContainer.CLIMBER_PIVOT_SPEED; // negative is up
                DriveFromControllerCommand.lockOnMode = false;
                break;

            case CLIMB_MANUAL:

                break;

            case TUNING:
                // used during tuning
                shooterRightRPM = SmartDashboard.getNumber("Shooter right power", 0);
                shooterLeftRPM = SmartDashboard.getNumber("Shooter left power", 0);
                pivotAngle = SmartDashboard.getNumber("Shooter angle", 0);
                pivotAngle = checkAngleLimits(pivotAngle);
                feederSpeed = SmartDashboard.getNumber("Feeder power", feederSpeed);
                pivotController.reset();
                pivotController.setSetpoint(pivotAngle);
                break;

        }
    }

    private double checkAngleLimits(double angle) {
        if (angle > PivotSubsystem.PIVOT_UPPER_ENDPOINT) {
            angle = PivotSubsystem.PIVOT_UPPER_ENDPOINT;
        }
        if (angle < PivotSubsystem.PIVOT_LOWER_ENDPOINT) {
            angle = PivotSubsystem.PIVOT_LOWER_ENDPOINT;
        }

        return angle;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        DriveFromControllerCommand.lockOnMode = false;
        feederSubsystem.setSpeed(0);
        shooterSubsystem.setShooterLeftRPM(0);
        shooterSubsystem.setShooterRightRPM(0);
        pivotSubsystem.setSpeed(0);
        System.out.println("Shooter Command Ending!!!!!!!!!!!");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        switch (mode) {
            case INTAKE:

                if (intakeSubsystem.isIntakeRunningForward() &&
                        feederDistanceSensorSubsystem.isNoteLoaded()) {
                    intakeSubsystem.setIntakeSpeed(0);
                    feederSubsystem.setSpeed(0);
                    mode = ACTION.NONE;
                    modeChanged = true;
                } else if (intakeLoadTrigger.get() < 0.5 && intakeSubsystem.isIntakeRunningForward()) {
                    intakeSubsystem.setIntakeSpeed(0);
                    feederSubsystem.setSpeed(0);
                    mode = ACTION.NONE;
                    modeChanged = true;
                }

                break;

            case EJECT:
                if (intakeEjectTrigger.get() < 0.5 && intakeSubsystem.isIntakeRunningBackward()) {
                    feederSubsystem.setSpeed(0);
                    intakeSubsystem.setIntakeSpeed(0);
                    mode = ACTION.NONE;
                    modeChanged = true;
                }

                break;

            case PARK:
                feederSubsystem.setSpeed(0);
                shooterSubsystem.setShooterLeftRPM(0);
                shooterSubsystem.setShooterRightRPM(0);
                DriveFromControllerCommand.lockOnMode = false;
                if (pivotController.atSetpoint()) {
                    pivotSubsystem.setSpeed(0);
                    mode = ACTION.NONE;
                    modeChanged = true;
                }

                break;

            case AMP:
            case SPEAKER:
            case TRAP:
            case TUNING:
                break;
            case FIRE:
                if (firing && timer.get() > RobotContainer.FEEDER_TIME) {
                    firing = false;
                    mode = ACTION.PARK;
                    modeChanged = true;
                }
                break;

            case NONE:
                break;

            case CLIMB_AUTO_LIFT:
                if (pivotSubsystem.getPivotAbsolutePosition() < pivotAngle) {
                    pivotSubsystem.setSpeed(0);
                    mode = ACTION.NONE;
                    modeChanged = true;
                }
                break;

            case CLIMB_AUTO_READY:
                if (pivotSubsystem.getPivotAbsolutePosition() > pivotAngle) {
                    pivotSubsystem.setSpeed(0);
                    mode = ACTION.NONE;
                    modeChanged = true;
                }
                break;

            case CLIMB_MANUAL:
                break;

        }

        return false;
    }

    private double computePivotAngle(double distance) {
        return 91.727 - 0.585 * distance + 0.00152 * distance * distance;
    }

    private double checkPivotSpeed(double speed) {
        if ((speed < 0 && pivotSubsystem.getPivotAbsolutePosition() > RobotContainer.PIVOT_UPPER_ENDPOINT)
                || (speed > 0 && pivotSubsystem.getPivotAbsolutePosition() < RobotContainer.PIVOT_LOWER_ENDPOINT)) {
            speed = 0;
        }

        return speed;
    }
}
