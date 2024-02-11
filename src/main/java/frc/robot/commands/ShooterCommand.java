package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
    private double leftShooterRPM;
    private double rightShooterRPM;
    private boolean autoAim = false;
    private double pivotAngle;
    private int direction;

    public ShooterCommand(FeederDistanceSensorSubsystem feederDistanceSensorSubsystem,
            ShooterSubsystem shooterSubsystem,
            PivotEncoderSubsystem pivotEncoderSubsystem,
            PivotSubsystem pivotSubsystem,
            double leftShooterRPM,
            double rightShooterRPM,
            boolean autoAim, double pivotAngle) {

        this.shooterSubsystem = shooterSubsystem;
        this.feederDistanceSensorSubsystem = feederDistanceSensorSubsystem;
        this.pivotEncoderSubsystem = pivotEncoderSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.leftShooterRPM = leftShooterRPM;
        this.rightShooterRPM = rightShooterRPM;
        this.autoAim = autoAim;
        this.pivotAngle = pivotAngle;

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
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (feederDistanceSensorSubsystem.isNoteLoaded()) {
            shooterSubsystem.setShooterLeftRPM(leftShooterRPM);
            shooterSubsystem.setShooterRightRPM(rightShooterRPM);
            if (pivotAngle != RobotContainer.AUTO_PIVOT_ANGLE) {
                pivotSubsystem.setSpeed(RobotContainer.PIVOT_SPEED * direction);
            } else {
                // TO-DO
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (pivotAngle != RobotContainer.AUTO_PIVOT_ANGLE) {
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
}
