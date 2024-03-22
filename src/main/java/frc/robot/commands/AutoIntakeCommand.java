package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends Command {

    private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private Timer timer;
    private double setTime;

    public AutoIntakeCommand(FeederDistanceSensorSubsystem feederDistanceSensorSubsystem,
            FeederSubsystem feederSubsystem,
            IntakeSubsystem intakeSubsystem,
            double setTime) {

        this.feederDistanceSensorSubsystem = feederDistanceSensorSubsystem;

        this.feederSubsystem = feederSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        timer = new Timer();
        this.setTime = setTime;

        addRequirements(feederDistanceSensorSubsystem);
        addRequirements(feederSubsystem);
        addRequirements(intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSubsystem.setIntakeSpeed(0.1); // RobotContainer.INTAKE_SPEED
        feederSubsystem.setSpeed(0.1); // RobotContainer.FEEDER_SPEED_INTAKE
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        feederSubsystem.setSpeed(0);
        intakeSubsystem.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        if (feederDistanceSensorSubsystem.isNoteLoaded() || timer.get() > setTime) {
            return true;
        } else {
            return false;
        }
    }

}