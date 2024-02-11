package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.PivotEncoderSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FireCommand extends Command {

    private final static double FEEDER_TIME = 2;

    private final ShooterSubsystem shooterSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final PivotEncoderSubsystem pivotEncoderSubsystem;
    private final PivotSubsystem pivotSubsystem;
    private final double feederRpm;
    private Timer timer;
    private int direction;
    private double pivotAngle;

    public FireCommand(FeederSubsystem feederSubsystem,
            ShooterSubsystem shooterSubsystem,
            PivotEncoderSubsystem pivotEncoderSubsystem,
            PivotSubsystem pivotSubsystem,
            double feederRpm) {

        this.shooterSubsystem = shooterSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.pivotEncoderSubsystem = pivotEncoderSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.feederRpm = feederRpm;

        timer = new Timer();

        pivotAngle = RobotContainer.SHOOTER_PIVOT_PARKED_ANGLE;
        if (pivotAngle > PivotEncoderSubsystem.PIVOT_UPPER_ENDPOINT) {
            pivotAngle = PivotEncoderSubsystem.PIVOT_UPPER_ENDPOINT;
        }
        if (pivotAngle < PivotEncoderSubsystem.PIVOT_LOWER_ENDPOINT) {
            pivotAngle = PivotEncoderSubsystem.PIVOT_LOWER_ENDPOINT;
        }

        addRequirements(shooterSubsystem);
        addRequirements(feederSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();

        // For pivot going back to parked position
        direction = 1; // going up
        if (pivotEncoderSubsystem.getPivotAbsolutePosition() > pivotAngle) {
            direction = -1; // going down
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (timer.get() < FEEDER_TIME) {
            feederSubsystem.setSpeed(feederRpm);
        } else {
            feederSubsystem.setSpeed(0);
            shooterSubsystem.setShooterLeftRPM(0);
            shooterSubsystem.setShooterRightRPM(0);
            pivotSubsystem.setSpeed(RobotContainer.PIVOT_SPEED * direction);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        feederSubsystem.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if ((direction == 1 && pivotEncoderSubsystem.getPivotAbsolutePosition() > pivotAngle)
                || (direction == -1 && pivotEncoderSubsystem.getPivotAbsolutePosition() < pivotAngle)) {
            pivotSubsystem.stop();
            return true;
        } else {
            return false;
        }
    }
}
