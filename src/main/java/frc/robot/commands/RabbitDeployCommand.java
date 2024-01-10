package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RabbitDeploySubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RabbitDeployCommand extends CommandBase {

    private final RabbitDeploySubsystem rabbitDeploySubsystem;
    private double position;
    public PIDController rabbitDeployPID = new PIDController(0.04, 0.003, 0);

    public RabbitDeployCommand(RabbitDeploySubsystem rabbitDeploySubsystem, double position) {
        this.rabbitDeploySubsystem = rabbitDeploySubsystem;
        this.position = position;
        rabbitDeployPID.setTolerance(0.01);
        // TO-DO limit position values
        addRequirements(rabbitDeploySubsystem);
    }

    @Override
    public void initialize() {
        /*direction = 1;

        if (rabbitDeploySubsystem.getEncoderPosition() > position) {
            direction = -1;
        }
        if (Math.abs(rabbitDeploySubsystem.getEncoderPosition() - position) < 3) {
            direction = 0;
        }*/
    }

    @Override
    public void execute() {
        double speed = rabbitDeployPID.calculate(rabbitDeploySubsystem.getEncoderPosition(), position);
        speed = MathUtil.clamp(speed, -0.8, 0.8);
        rabbitDeploySubsystem.setDeploySpeed(speed);
        SmartDashboard.putNumber("Rabbit Deploy Encoder", rabbitDeploySubsystem.getEncoderPosition());
    }

    @Override
    public void end(boolean interrupted) {
        rabbitDeploySubsystem.setDeploySpeed(0);
        System.out.println(rabbitDeploySubsystem.getEncoderPosition());
    }

    @Override
    public boolean isFinished() {
        
        return rabbitDeployPID.atSetpoint();
    }
    
}
