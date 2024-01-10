package frc.robot.commands;

import frc.robot.subsystems.BallShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BallShooterCommand extends CommandBase{
    private final BallShooterSubsystem ballShooterSubsystem;
    private static final double BALL_SHOOTER_SPEED = .2;
    private double rotationAmount;

    public BallShooterCommand(BallShooterSubsystem ballShooterSubsystem, double rotationAmount) {
        this.ballShooterSubsystem = ballShooterSubsystem;
        this.rotationAmount = rotationAmount;
        addRequirements(ballShooterSubsystem);
    }

    @Override
    public void initialize() {
        ballShooterSubsystem.resetEncoder();
    }

    @Override
    public void execute() {
        ballShooterSubsystem.setSpeed(BALL_SHOOTER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        ballShooterSubsystem.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        if (ballShooterSubsystem.getEncoderPosition() >= rotationAmount) {
            return true;
        }

        return false;
    }
}
