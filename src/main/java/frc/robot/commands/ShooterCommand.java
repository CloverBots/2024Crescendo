package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionTargetTracker;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.PivotEncoderSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem;
    private final PivotEncoderSubsystem pivotEncoderSubsystem;
    private final PivotSubsystem pivotSubsystem;
    private final VisionTargetTracker visionTargetTracker;
    private double leftShooterRPM;
    private double rightShooterRPM;
    private boolean autoAim = false;
    private boolean autoPivotAngle = false;
    private double pivotAngle;
    private double computedPivotAngle;
    private int direction;

    public ShooterCommand(FeederDistanceSensorSubsystem feederDistanceSensorSubsystem,
            ShooterSubsystem shooterSubsystem,
            PivotEncoderSubsystem pivotEncoderSubsystem,
            PivotSubsystem pivotSubsystem,
            VisionTargetTracker visionTargetTracker,
            double leftShooterRPM,
            double rightShooterRPM,
            boolean autoAim, double pivotAngle) {

        this.shooterSubsystem = shooterSubsystem;
        this.feederDistanceSensorSubsystem = feederDistanceSensorSubsystem;
        this.pivotEncoderSubsystem = pivotEncoderSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.visionTargetTracker = visionTargetTracker;
        this.leftShooterRPM = leftShooterRPM;
        this.rightShooterRPM = rightShooterRPM;
        this.autoAim = autoAim;
        this.pivotAngle = pivotAngle;

        if (pivotAngle == RobotContainer.AUTO_PIVOT_ANGLE) {
            autoPivotAngle = true;
        }

        if (pivotAngle > PivotEncoderSubsystem.PIVOT_UPPER_ENDPOINT) {
            pivotAngle = PivotEncoderSubsystem.PIVOT_UPPER_ENDPOINT;
        }
        if (pivotAngle < PivotEncoderSubsystem.PIVOT_LOWER_ENDPOINT) {
            pivotAngle = PivotEncoderSubsystem.PIVOT_LOWER_ENDPOINT;
        }

        addRequirements(shooterSubsystem);
        addRequirements(feederDistanceSensorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        direction = 1; // going up
        if (pivotEncoderSubsystem.getPivotAbsolutePosition() > pivotAngle) {
            direction = -1; // going down
        }
        if (autoAim) DriveFromControllerCommand.lockOnMode = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // If note is loaded or going back to parked position
        if (feederDistanceSensorSubsystem.isNoteLoaded() ||
                pivotAngle == RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE) {

            shooterSubsystem.setShooterLeftRPM(leftShooterRPM);
            shooterSubsystem.setShooterRightRPM(rightShooterRPM);

            // for auto mode,
            // base the pivot angle on the Apriltag distance
            if (autoPivotAngle) {
                double targetDistance = visionTargetTracker.computeTargetDistance();
                Boolean isTargetValid = visionTargetTracker.isValid();
                if (isTargetValid) {
                    computedPivotAngle = computePivotAngle(targetDistance);
                    //TO-DO move pivot (determine direction, move)
                } else {
                    //TO-DO some default setting?
                }
            } else {
                pivotSubsystem.setSpeed(RobotContainer.PIVOT_SPEED * direction);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (autoAim) DriveFromControllerCommand.lockOnMode = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //For autoPivotAngle == true, keep running command so the angle will continuously update
        if (!autoPivotAngle) {
            if ((direction == 1 && pivotEncoderSubsystem.getPivotAbsolutePosition() > pivotAngle)
                    || (direction == -1 && pivotEncoderSubsystem.getPivotAbsolutePosition() < pivotAngle)) {
                pivotSubsystem.stop();
                return true;
            } else {
                return false;
            }
        }
        return false;
    }

    private double computePivotAngle(double distance) {
        return 46.7 * distance + 1793; //TO-DO determine proper formula
    }
}
