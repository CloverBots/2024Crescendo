package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallIntakeSubsystem;

public class BallIntakeCommand extends CommandBase {
    private final DoubleSupplier inSpeed;
    private final DoubleSupplier outSpeed;
    private final BallIntakeSubsystem ballIntakeSubsystem;

    public BallIntakeCommand(BallIntakeSubsystem ballIntakeSubsystem, DoubleSupplier inSpeed, DoubleSupplier outSpeed) {
        this.ballIntakeSubsystem = ballIntakeSubsystem;
        this.inSpeed = inSpeed;
        this.outSpeed = outSpeed;
        addRequirements(ballIntakeSubsystem);
    }

    @Override
    public void execute() {
        double in = inSpeed.getAsDouble();
        double out = outSpeed.getAsDouble();
        double speed = 0;

        if (in > 0.05 && out < 0.05) {
            speed = in;
        } else if (in < 0.05 && out > 0.05) {
            speed = -out;
        } else {
            speed = 0;
        }
        ballIntakeSubsystem.setIntakeSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        ballIntakeSubsystem.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}