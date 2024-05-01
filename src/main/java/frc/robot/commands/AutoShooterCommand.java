package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShooterCommand extends Command {

    private final PivotSubsystem pivotSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final double pivotAngle;
    private final double shooterLeftRPM;
    private final double shooterRightRPM;

    public AutoShooterCommand(PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, double pivotAngle,
            double shooterLeftRPM, double shooterRightRPM) {
        this.pivotSubsystem = pivotSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.pivotAngle = pivotAngle;
        this.shooterLeftRPM = shooterLeftRPM;
        this.shooterRightRPM = shooterRightRPM;

        addRequirements(pivotSubsystem);
        addRequirements(shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pivotSubsystem.setPivotControllerSetpoint(pivotAngle);
        shooterSubsystem.setShooterLeftRPM(shooterLeftRPM);
        shooterSubsystem.setShooterRightRPM(shooterRightRPM);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.setPivotControllerSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE);
        shooterSubsystem.setShooterLeftRPM(0);
        shooterSubsystem.setShooterRightRPM(0);
    }

    @Override
    public boolean isFinished() {
        if (pivotSubsystem.pivotReady()) {
            return true;
        } else {
            return false;
        }
    }

}