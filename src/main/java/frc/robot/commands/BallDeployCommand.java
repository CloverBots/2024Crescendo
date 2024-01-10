package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallDeploySubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallDeployCommand extends CommandBase{
    private final BallDeploySubsystem ballDeploySubsystem;
    private double position;
    private int direction;
    private double speed;

    public BallDeployCommand(BallDeploySubsystem ballDeploySubsystem, double position, double speed) {
        this.ballDeploySubsystem = ballDeploySubsystem;
        this.position = position;
        this.speed = speed;
        // TO-DO limit position values
        addRequirements(ballDeploySubsystem);
    }

    @Override
    public void initialize() {
        direction = 1;

        if (ballDeploySubsystem.getEncoderPosition() > position) {
            direction = -1;
        }
        if (Math.abs(ballDeploySubsystem.getEncoderPosition() - position) < 3) {
            direction = 0;
        }
    }

    @Override
    public void execute() {
        ballDeploySubsystem.setDeploySpeed(speed * direction);
    }

    @Override
    public void end(boolean interrupted) {
        ballDeploySubsystem.setDeploySpeed(0);
        System.out.println(ballDeploySubsystem.getEncoderPosition());
    }

    @Override
    public boolean isFinished() {
        if (direction == -1 && ballDeploySubsystem.getEncoderPosition() <= position) {
            return true;
        } else if (direction == 1 && ballDeploySubsystem.getEncoderPosition() >= position) {
            return true;
        } else if (direction == 0) {
            return true;
        }

        return false;
    }
    
}
