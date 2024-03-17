package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionTargetTracker;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem;
    private final LEDSubsystem ledSubsystem;
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

    private double previousVisionTx, previousVisionTy;

    private boolean firing = false;
    private boolean noteLoaded = false;
    private boolean readyToFire = false;

    public enum ACTION {
        NONE, INTAKE, EJECT, AMP, SPEAKER, DEFAULT_SPEAKER, TRAP, FIRE, PARK, TUNING, CLIMB_AUTO_READY, CLIMB_AUTO_LIFT,
        CLIMB_MANUAL
    }

    private ACTION mode;
    private boolean modeChanged = false;

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
        ledSubsystem = new LEDSubsystem();

        timer = new Timer();
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
        pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE);
        pivotSubsystem.enable();
        SmartDashboard.putBoolean("Camera", true);
        ledSubsystem.setLEDChannelB();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

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
                    intakeSubsystem.setIntakeSpeed(RobotContainer.INTAKE_SPEED);
                    feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_INTAKE);
                }

                break;
            case EJECT:
                intakeSubsystem.setIntakeSpeed(-RobotContainer.INTAKE_SPEED);
                feederSubsystem.setSpeed(-RobotContainer.FEEDER_SPEED_INTAKE);
                break;

            case TUNING:
                shooterRightRPM = SmartDashboard.getNumber("Shooter right RPM", 0);
                shooterLeftRPM = SmartDashboard.getNumber("Shooter left RPM", 0);
                pivotAngle = SmartDashboard.getNumber("Shooter angle", 50);
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotSubsystem.setPivotControllerSetpoint(pivotAngle);

            case AMP:
            case TRAP:
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
                    DriveFromControllerCommand.lockOnMode = true;
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
                ledSubsystem.setLEDChannelB();
                break;

            case CLIMB_AUTO_LIFT:
            case CLIMB_AUTO_READY:
                pivotSubsystem.setSpeed(pivotSpeed);
                break;

            case CLIMB_MANUAL:
                if (Math.abs(leftJoystickY.getAsDouble()) > .05) {
                    pivotSpeed = -leftJoystickY.getAsDouble() / 2.2; // - is because joystick returns 1 for down, -1 for
                                                                   // up

                    if (pivotSpeed > 0.05 &&
                            pivotSubsystem.getPivotAbsolutePosition() > RobotContainer.PIVOT_UPPER_ENDPOINT) {
                        pivotSpeed = 0;
                    }

                    if (pivotSpeed < -.05 &&
                            pivotSubsystem.getPivotAbsolutePosition() < RobotContainer.PIVOT_LOWER_ENDPOINT) {
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
        } else if (bButton.get() && mode != ACTION.SPEAKER) {
            mode = ACTION.SPEAKER;
            modeChanged = true;
        } else if (yButton.get() && mode != ACTION.TRAP) {
            // re-purposing Y button mode = ACTION.TRAP;
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
        } else if (dPad.get() == 0) { // disabled
            // mode = ACTION.CLIMB_AUTO_READY;
            // modeChanged = true;
        } else if (dPad.get() == 180) { // disabled
            // mode = ACTION.CLIMB_AUTO_LIFT;
            // modeChanged = true;
        }
    }

    private void updateValues() {
        modeChanged = false;

        switch (mode) {
            case INTAKE:
            case EJECT:
                shooterSubsystem.setDefaultShooterRPM();
                feederSubsystem.setSpeed(0);
                pivotAngle = RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotSubsystem.setPivotControllerSetpoint(pivotAngle);

                break;

            case PARK:
                shooterLeftRPM = 0;
                shooterRightRPM = 0;
                shooterSubsystem.setDefaultShooterRPM();
                feederSpeed = 0;
                pivotAngle = RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotSubsystem.setPivotControllerSetpoint(pivotAngle);
                DriveFromControllerCommand.lockOnMode = false;
                break;

            case AMP:
                shooterLeftRPM = RobotContainer.SHOOTER_AMP_LEFT_RPM;
                shooterRightRPM = RobotContainer.SHOOTER_AMP_RIGHT_RPM;
                feederSpeed = RobotContainer.FEEDER_SPEED_SHOOT;
                pivotAngle = RobotContainer.SHOOTER_AMP_PIVOT_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotSubsystem.setPivotControllerSetpoint(pivotAngle);
                break;

            case SPEAKER:
                if (visionTargetTracker.isValid() && (previousVisionTy != visionTargetTracker.getTy()
                        || previousVisionTx != visionTargetTracker.getTx())) {

                    SmartDashboard.putBoolean("Camera", true);

                } else if (visionTargetTracker.isValid()) {
                    SmartDashboard.putBoolean("Camera", false);

                }

                shooterLeftRPM = RobotContainer.SHOOTER_SPEAKER_LEFT_RPM;
                shooterRightRPM = RobotContainer.SHOOTER_SPEAKER_RIGHT_RPM;
                feederSpeed = RobotContainer.FEEDER_SPEED_SHOOT;
                pivotAngle = RobotContainer.SHOOTER_SPEAKER_PIVOT_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                previousPivotAngle = pivotAngle;
                pivotSubsystem.setPivotControllerSetpoint(pivotAngle);
                previousPivotAngle = pivotAngle;

                break;

            case DEFAULT_SPEAKER:
                if (visionTargetTracker.isValid() && (previousVisionTy != visionTargetTracker.getTy()
                        || previousVisionTx != visionTargetTracker.getTx())) {
                    // Looks like camera is working again!
                    SmartDashboard.putBoolean("Camera", true);

                }

                shooterLeftRPM = RobotContainer.SHOOTER_SPEAKER_LEFT_RPM;
                shooterRightRPM = RobotContainer.SHOOTER_SPEAKER_RIGHT_RPM;
                feederSpeed = RobotContainer.FEEDER_SPEED_SHOOT;
                pivotAngle = RobotContainer.SHOOTER_SPEAKER_PIVOT_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                previousPivotAngle = pivotAngle;
                pivotSubsystem.setPivotControllerSetpoint(pivotAngle);

                break;

            case TRAP:
                shooterLeftRPM = RobotContainer.SHOOTER_TRAP_LEFT_RPM;
                shooterRightRPM = RobotContainer.SHOOTER_TRAP_RIGHT_RPM;
                feederSpeed = RobotContainer.FEEDER_SPEED_SHOOT;
                pivotAngle = RobotContainer.SHOOTER_TRAP_PIVOT_ANGLE;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotSubsystem.setPivotControllerSetpoint(pivotAngle);
                break;

            case FIRE:
                feederSpeed = RobotContainer.FEEDER_SPEED_SHOOT;

                // Update previous values for comparison during next shot
                if (visionTargetTracker.isValid()) {
                    previousVisionTy = visionTargetTracker.getTy();
                    previousVisionTx = visionTargetTracker.getTx();
                }

            case NONE:

                break;

            case CLIMB_AUTO_LIFT:
                shooterLeftRPM = 0;
                shooterRightRPM = 0;
                feederSpeed = 0;
                pivotSubsystem.disable();
                pivotAngle = RobotContainer.CLIMBER_RAISED_POSITION;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotSpeed = -RobotContainer.CLIMBER_PIVOT_SPEED;
                DriveFromControllerCommand.lockOnMode = false;
                break;

            case CLIMB_AUTO_READY:
                shooterLeftRPM = 0;
                shooterRightRPM = 0;
                feederSpeed = 0;
                pivotSubsystem.disable();
                pivotAngle = RobotContainer.CLIMBER_READY_POSITION;
                pivotAngle = checkAngleLimits(pivotAngle);
                pivotSpeed = RobotContainer.CLIMBER_PIVOT_SPEED; // negative is up
                DriveFromControllerCommand.lockOnMode = false;
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
                    ledSubsystem.setLEDChannelA();
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
                    ledSubsystem.setLEDChannelB();
                }

                break;

            case PARK:
                feederSubsystem.setSpeed(0);
                shooterSubsystem.setDefaultShooterRPM();
                DriveFromControllerCommand.lockOnMode = false;
                if (pivotSubsystem.atSetpoint()) {
                    mode = ACTION.NONE;
                    modeChanged = true;
                }

                break;

            case AMP:
            case SPEAKER:
            case DEFAULT_SPEAKER:
            case TRAP:
            case TUNING:
                if (shooterSubsystem.isShooterAtTargetRpm() && pivotSubsystem.pivotReady() && noteLoaded) {
                    readyToFire = true;
                    ledSubsystem.setLEDChannelC();
                } else {
                    readyToFire = false;
                }
                break;
            case FIRE:
                if (firing && timer.get() > RobotContainer.FEEDER_TIME) {
                    firing = false;
                    readyToFire = false;
                    ledSubsystem.setLEDChannelB();
                    mode = ACTION.PARK;
                    DriveFromControllerCommand.lockOnMode = false;
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

}
