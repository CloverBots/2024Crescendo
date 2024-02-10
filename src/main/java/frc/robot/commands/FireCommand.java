package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotEncoderSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FireCommand extends Command {
    private final static double PIVOT_SPEED = 0.2;

    private final ShooterSubsystem shooterSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem;
    private final PivotEncoderSubsystem pivotEncoderSubsystem;
    private final PivotSubsystem pivotSubsystem;
    private final double feederRpm;

    public FireCommand(FeederSubsystem feederSubsystem,
            FeederDistanceSensorSubsystem feederDistanceSensorSubsystem,
            ShooterSubsystem shooterSubsystem,
            PivotEncoderSubsystem pivotEncoderSubsystem,
            PivotSubsystem pivotSubsystem,
            double feederRpm) {

        this.shooterSubsystem = shooterSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.feederDistanceSensorSubsystem = feederDistanceSensorSubsystem;
        this.pivotEncoderSubsystem = pivotEncoderSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.feederRpm = feederRpm; 


        addRequirements(shooterSubsystem);
        addRequirements(feederSubsystem);
        addRequirements(feederDistanceSensorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        feederSubsystem.setSpeed(feederRpm);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
