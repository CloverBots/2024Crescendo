package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends Command {

    private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public AutoIntakeCommand(FeederDistanceSensorSubsystem feederDistanceSensorSubsystem,
            FeederSubsystem feederSubsystem,
            IntakeSubsystem intakeSubsystem) {
        this.feederDistanceSensorSubsystem = feederDistanceSensorSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(feederDistanceSensorSubsystem);
        addRequirements(feederSubsystem);
        addRequirements(intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSubsystem.setIntakeSpeed(RobotContainer.INTAKE_SPEED);
        feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_INTAKE);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        feederSubsystem.setSpeed(0);
        intakeSubsystem.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        if (feederDistanceSensorSubsystem.isNoteLoaded()) {
            return true;
        } else {
            return false;
        }
    }

}