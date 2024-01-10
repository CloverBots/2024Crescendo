package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RabbitIntakeSubsystem;

public class RabbitIntakeCommand extends CommandBase{
    private final RabbitIntakeSubsystem rabbitIntakeSubsystem;
    private final DoubleSupplier leftJoystickY;

    public RabbitIntakeCommand(RabbitIntakeSubsystem rabbitIntakeSubsystem, DoubleSupplier leftJoystickY) {
        this.rabbitIntakeSubsystem = rabbitIntakeSubsystem;
        this.leftJoystickY = leftJoystickY;
        addRequirements(rabbitIntakeSubsystem);
    }

    @Override
    public void execute() {
        double speed = -leftJoystickY.getAsDouble();

        if (Math.abs(speed) < 0.05) {
            speed = 0;
        }

        rabbitIntakeSubsystem.setIntakeSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        rabbitIntakeSubsystem.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
