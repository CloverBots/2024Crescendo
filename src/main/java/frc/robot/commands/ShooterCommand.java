package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

    private static final double SHOOTER_RIGHT_RPM = 20;
    private static final double SHOOTER_LEFT_RPM = 20;
    private static final double FEEDER_RPM = 20;

    private final ShooterSubsystem shooterSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem;

    private final Supplier<Boolean> shooterPrepareButton, shootButton;

    private boolean shotFired = false;

    public ShooterCommand(FeederSubsystem feederSubsystem,
            FeederDistanceSensorSubsystem feederDistanceSensorSubsystem,
            ShooterSubsystem shooterSubsystem,
            Supplier<Boolean> shooterPrepareButton,
            Supplier<Boolean> shootButton) {

        this.shooterSubsystem = shooterSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.feederDistanceSensorSubsystem = feederDistanceSensorSubsystem;

        this.shooterPrepareButton = shooterPrepareButton;
        this.shootButton = shootButton;

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
        if (shooterPrepareButton.get() && feederDistanceSensorSubsystem.isNoteLoaded()) {
            shooterSubsystem.setShooterLeftRPM(SHOOTER_LEFT_RPM);
            shooterSubsystem.setShooterRightRPM(SHOOTER_RIGHT_RPM);
        } else if (!shooterPrepareButton.get()) {
            shooterSubsystem.setShooterLeftRPM(0);
            shooterSubsystem.setShooterRightRPM(0);
        }

        if (shootButton.get() && shooterSubsystem.isShooterAtTargetRpm()) {
            feederSubsystem.setSpeed(FEEDER_RPM);
            shotFired = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShooterLeftRPM(0);
        shooterSubsystem.setShooterRightRPM(0);
        feederSubsystem.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (shotFired && !feederDistanceSensorSubsystem.isNoteLoaded()) {
            feederSubsystem.setSpeed(0);
        }

        return false;
    }
}
