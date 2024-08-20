package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.LEDs.LEDSubsystem.State;
import limelight.LimelightTargetTracking;

public class ShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem;
    private final LEDSubsystem ledSubsystem;
    private final PivotSubsystem pivotSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final LimelightTargetTracking visionTargetTracker;
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
    private Timer matchTime;
    private boolean blinkOn = false;

    private double previousVisionTx, previousVisionTy;

    private boolean firing = false;
    private boolean noteLoaded = false;
    private boolean readyToFire = false;

    public enum ACTION {
        NONE, INTAKE, EJECT, AMP, SPEAKER, DEFAULT_SPEAKER, FIRE, PARK, TUNING, LOB_HIGH, LOB_LOW,
        CLIMB_MANUAL
    }

    private ACTION mode;
    private boolean modeChanged = false;

    public ShooterCommand(FeederDistanceSensorSubsystem feederDistanceSensorSubsystem,
            ShooterSubsystem shooterSubsystem,
            PivotSubsystem pivotSubsystem,
            FeederSubsystem feederSubsystem,
            IntakeSubsystem intakeSubsystem,
            LimelightTargetTracking visionTargetTracker,
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
        ledSubsystem = LEDSubsystem.getInstance();

        timer = new Timer();
        matchTime = new Timer();

        mode = ACTION.NONE;

        addRequirements(shooterSubsystem);
        addRequirements(feederDistanceSensorSubsystem);
        addRequirements(pivotSubsystem);
        addRequirements(feederSubsystem);
        addRequirements(intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pivotSubsystem.setPivotControllerSetpoint(PivotConstants.PIVOT_PARKED_ANGLE);
        pivotSubsystem.enable();
        // SmartDashboard.putBoolean("Camera", true);
        matchTime.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (matchTime.get() > 110) {
            matchTime.stop();
            matchTime.reset();
            blinkOn = true;
        }

        SmartDashboard.putBoolean("Note Loaded", noteLoaded);

        SmartDashboard.putBoolean("Fire", readyToFire);

        checkControllerValues();

        // If mode changed then get appropriate values for the new mode
        if (modeChanged) {
            updateValues();
        }

        switch (mode) {
            case INTAKE:
                if (!noteLoaded) {
                    intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
                    feederSubsystem.setSpeed(IntakeConstants.FEEDER_SPEED_INTAKE);
                }

                break;
            case EJECT:
                intakeSubsystem.setIntakeSpeed(-IntakeConstants.INTAKE_SPEED);
                feederSubsystem.setSpeed(-IntakeConstants.FEEDER_SPEED_INTAKE);
                break;

            case TUNING:
            case AMP:
            case LOB_LOW:
            case LOB_HIGH:
            case DEFAULT_SPEAKER:

                shooterSubsystem.setShooterLeftRPM(shooterLeftRPM);
                shooterSubsystem.setShooterRightRPM(shooterRightRPM);

                break;

            case PARK:

                break;

            case SPEAKER:

                double targetDistance = 0;

                if (visionTargetTracker.isValid()) {
                    targetDistance = visionTargetTracker.computeTargetDistance();
                    pivotAngle = visionTargetTracker.computePivotAngle(targetDistance);
                    pivotAngle = checkAngleLimits(pivotAngle);
                    DriveCommand.lockOnMode = true;
                }

                // If the desired angle has changed by 1 degree or more, update the setpoint
                if (Math.abs(previousPivotAngle - pivotAngle) > 1) {
                    previousPivotAngle = pivotAngle;
                    pivotSubsystem.setPivotControllerSetpoint(pivotAngle);

                    shooterLeftRPM = visionTargetTracker.computeShooterLeftSpeed(targetDistance);
                    shooterRightRPM = visionTargetTracker.computeShooterRightSpeed(targetDistance);
                }

                shooterSubsystem.setShooterLeftRPM(shooterLeftRPM);
                shooterSubsystem.setShooterRightRPM(shooterRightRPM);
                ledSubsystem.conformToState(State.SOLID_PINK);

                SmartDashboard.putNumber("target distance", visionTargetTracker.computeTargetDistance());
                break;

            case FIRE:
                if (!firing) {
                    firing = true;
                    noteLoaded = false;
                    timer.reset();
                    timer.start();
                    feederSubsystem.setSpeed(feederSpeed);
                }
                break;

            case NONE:
                ledConformToState("Green");
                break;

            case CLIMB_MANUAL:
                if (Math.abs(leftJoystickY.getAsDouble()) > .05) {
                    pivotSpeed = -leftJoystickY.getAsDouble() / 2.2; // - is because joystick returns 1 for down, -1 for
                                                                     // up

                    if (pivotSpeed > 0.05 &&
                            pivotSubsystem.getPivotAbsolutePosition() > PivotConstants.PIVOT_UPPER_ENDPOINT) {
                        pivotSpeed = 0;
                    }

                    if (pivotSpeed < -.05 &&
                            pivotSubsystem.getPivotAbsolutePosition() < PivotConstants.PIVOT_LOWER_ENDPOINT) {
                        pivotSpeed = 0;
                    }

                    if (Math.abs(pivotSpeed) < 0.2) {
                        pivotSpeed = pivotSpeed / 2;
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
            ledSubsystem.conformToState(State.SOLID_YELLOW);
        } else if (bButton.get() && mode != ACTION.SPEAKER) {
            mode = ACTION.SPEAKER;
            modeChanged = true;
        } else if (yButton.get() && mode != ACTION.DEFAULT_SPEAKER) {
            mode = ACTION.DEFAULT_SPEAKER;
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
            ledSubsystem.conformToState(State.SOLID_PURPLE);
        } else if (dPad.get() == 0) { // 0 is up on dpad
            mode = ACTION.LOB_HIGH;
            modeChanged = true;
            ledSubsystem.conformToState(State.SOLID_ORANGE);
        } else if (dPad.get() == 180) {
            mode = ACTION.LOB_LOW;
            modeChanged = true;
            ledSubsystem.conformToState(State.SOLID_ORANGE);
        }
    }

    private void updateValues() {
        modeChanged = false;

        switch (mode) {
            case INTAKE:
            case EJECT:
                shooterSubsystem.setDefaultShooterRPM();
                feederSubsystem.setSpeed(0);
                pivotAngle = PivotConstants.PIVOT_PARKED_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotSubsystem.setPivotControllerSetpoint(pivotAngle);
                DriveCommand.lockOnMode = false;

                break;

            case PARK:
                shooterLeftRPM = 0;
                shooterRightRPM = 0;
                shooterSubsystem.setDefaultShooterRPM();
                feederSpeed = 0;
                pivotAngle = PivotConstants.PIVOT_PARKED_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotSubsystem.setPivotControllerSetpoint(pivotAngle);
                DriveCommand.lockOnMode = false;
                break;

            case AMP:
                shooterLeftRPM = ShooterConstants.SHOOTER_AMP_LEFT_RPM;
                shooterRightRPM = ShooterConstants.SHOOTER_AMP_RIGHT_RPM;
                feederSpeed = IntakeConstants.FEEDER_SPEED_SHOOT;
                pivotAngle = PivotConstants.PIVOT_AMP_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotSubsystem.setPivotControllerSetpoint(pivotAngle);
                break;

            case SPEAKER:
                if (visionTargetTracker.isValid() && (previousVisionTy != visionTargetTracker.getTy()
                        || previousVisionTx != visionTargetTracker.getTx())) {

                    // SmartDashboard.putBoolean("Camera", true);

                } else if (visionTargetTracker.isValid()) {
                    // SmartDashboard.putBoolean("Camera", false);

                }

                shooterLeftRPM = ShooterConstants.SHOOTER_SPEAKER_LEFT_RPM;
                shooterRightRPM = ShooterConstants.SHOOTER_SPEAKER_RIGHT_RPM;
                feederSpeed = IntakeConstants.FEEDER_SPEED_SHOOT;
                pivotAngle = PivotConstants.PIVOT_PARKED_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                previousPivotAngle = pivotAngle;
                pivotSubsystem.setPivotControllerSetpoint(pivotAngle);
                previousPivotAngle = pivotAngle;

                break;

            case DEFAULT_SPEAKER:
                if (visionTargetTracker.isValid() && (previousVisionTy != visionTargetTracker.getTy()
                        || previousVisionTx != visionTargetTracker.getTx())) {
                }

                shooterLeftRPM = ShooterConstants.SHOOTER_SPEAKER_LEFT_RPM;
                shooterRightRPM = ShooterConstants.SHOOTER_SPEAKER_RIGHT_RPM;
                feederSpeed = IntakeConstants.FEEDER_SPEED_SHOOT;
                pivotAngle = PivotConstants.PIVOT_SPEAKER_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                previousPivotAngle = pivotAngle;
                pivotSubsystem.setPivotControllerSetpoint(pivotAngle);

                break;

            case FIRE:
                feederSpeed = IntakeConstants.FEEDER_SPEED_SHOOT;

                // Update previous values for comparison during next shot
                if (visionTargetTracker.isValid()) {
                    previousVisionTy = visionTargetTracker.getTy();
                    previousVisionTx = visionTargetTracker.getTx();
                }

            case NONE:

                break;

            case LOB_LOW:
                shooterLeftRPM = ShooterConstants.SHOOTER_UNDER_STAGE_LEFT_RPM;
                shooterRightRPM = ShooterConstants.SHOOTER_UNDER_STAGE_RIGHT_RPM;
                feederSpeed = IntakeConstants.FEEDER_SPEED_SHOOT;
                pivotAngle = PivotConstants.PIVOT_UNDER_STAGE_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotSubsystem.setPivotControllerSetpoint(pivotAngle);
                break;

            case LOB_HIGH:
                shooterLeftRPM = ShooterConstants.SHOOTER_OVER_STAGE_LEFT_RPM;
                shooterRightRPM = ShooterConstants.SHOOTER_OVER_STAGE_RIGHT_RPM;
                feederSpeed = IntakeConstants.FEEDER_SPEED_SHOOT;
                pivotAngle = PivotConstants.PIVOT_OVER_STAGE_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotSubsystem.setPivotControllerSetpoint(pivotAngle);
                break;

            case CLIMB_MANUAL:
                pivotSubsystem.disable();
                break;

            case TUNING:
                // used during tuning
                shooterRightRPM = SmartDashboard.getNumber("Shooter right RPM", 0);
                shooterLeftRPM = SmartDashboard.getNumber("Shooter left RPM", 0);
                pivotAngle = SmartDashboard.getNumber("Shooter angle", 0);
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotSubsystem.setPivotControllerSetpoint(pivotAngle);
                break;

        }
    }

    private double checkAngleLimits(double angle) {
        if (angle > PivotConstants.PIVOT_UPPER_ENDPOINT) {
            angle = PivotConstants.PIVOT_UPPER_ENDPOINT;
        }
        if (angle < PivotConstants.PIVOT_LOWER_ENDPOINT) {
            angle = PivotConstants.PIVOT_LOWER_ENDPOINT;
        }

        return angle;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        DriveCommand.lockOnMode = false;
        feederSubsystem.setSpeed(0);
        intakeSubsystem.setIntakeSpeed(0);
        shooterSubsystem.setShooterLeftRPM(0);
        shooterSubsystem.setShooterRightRPM(0);
        pivotSubsystem.setSpeed(0);
        pivotSubsystem.disable();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        switch (mode) {
            case INTAKE:

                if (feederDistanceSensorSubsystem.isNoteLoaded()) {
                    intakeSubsystem.setIntakeSpeed(0);
                    feederSubsystem.setSpeed(0);
                    ledConformToState("Red");
                    mode = ACTION.NONE;
                    modeChanged = true;
                    noteLoaded = true;
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
                    noteLoaded = false;
                    ledConformToState("Green");
                }

                break;

            case PARK:
                feederSubsystem.setSpeed(0);
                shooterSubsystem.setDefaultShooterRPM();
                DriveCommand.lockOnMode = false;
                if (pivotSubsystem.atSetpoint()) {
                    mode = ACTION.NONE;
                    modeChanged = true;
                }

                break;

            case AMP:
            case SPEAKER:
            case DEFAULT_SPEAKER:
            case LOB_LOW:
            case LOB_HIGH:
            case TUNING:
                if (shooterSubsystem.isShooterAtTargetRpm() && pivotSubsystem.pivotReady() && noteLoaded) {
                    readyToFire = true;
                    ledConformToState("Blue");
                } else {
                    readyToFire = false;
                }
                break;
            case FIRE:
                if (firing && timer.get() > IntakeConstants.FEEDER_TIME) {
                    firing = false;
                    readyToFire = false;
                    ledConformToState("Green");
                    mode = ACTION.PARK;
                    DriveCommand.lockOnMode = false;
                    modeChanged = true;
                }
                break;

            case NONE:
                break;

            case CLIMB_MANUAL:
                break;

        }

        return false;
    }

    private void ledConformToState(String color) {
        if (blinkOn) {
            if (color.equals("Blue")) {
                ledSubsystem.conformToState(State.FLASHING_BLUE);
            } else if (color.equals("Red")) {
                ledSubsystem.conformToState(State.FLASHING_RED);
            } else if (color.equals("Green")) {
                ledSubsystem.conformToState(State.FLASHING_GREEN);
            }
        } else {
            if (color.equals("Blue")) {
                ledSubsystem.conformToState(State.SOLID_BLUE);
            } else if (color.equals("Red")) {
                ledSubsystem.conformToState(State.SOLID_RED);
            } else if (color.equals("Green")) {
                ledSubsystem.conformToState(State.SOLID_GREEN);
            }
        }
    }
}