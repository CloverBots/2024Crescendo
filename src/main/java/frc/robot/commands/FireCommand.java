package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FireCommand extends Command {

    private final static double FEEDER_TIME = 2;

    private final FeederSubsystem feederSubsystem;
    private final double feederRpm;
    private Timer timer;
    private double pivotAngle;

    public FireCommand(FeederSubsystem feederSubsystem,
            double feederRpm) {

        this.feederSubsystem = feederSubsystem;
        this.feederRpm = feederRpm;

        timer = new Timer();

        addRequirements(feederSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.get() < FEEDER_TIME) {
            feederSubsystem.setSpeed(feederRpm);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        feederSubsystem.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (timer.get() >= FEEDER_TIME) {
            return true;
        } else {
        return false;
        }
    }
}
