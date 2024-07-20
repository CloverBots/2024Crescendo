// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooter extends Command {
    /** Creates a new RunIntake. */
    public final ShooterSubsystem shooterSubsystem;
    public final PivotSubsystem pivotSubsystem;
    public final FeederSubsystem feederSubsystem;
    public final int leftShooterRPM;
    public final int rightShooterRPM;
    public final double pivotSetPoint;
    public Timer timer = new Timer();
    public boolean isFiring = false;

    public RunShooter(ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, FeederSubsystem feederSubsystem, int leftShooterRPM, int rightShooterRPM, double pivotSetPoint) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooterSubsystem = shooterSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.leftShooterRPM = leftShooterRPM;
        this.rightShooterRPM = rightShooterRPM;
        this.pivotSetPoint = pivotSetPoint;

        addRequirements(shooterSubsystem);
        addRequirements(pivotSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterSubsystem.setShooterLeftRPM(leftShooterRPM);
        shooterSubsystem.setShooterRightRPM(rightShooterRPM);
        pivotSubsystem.setPivotControllerSetpoint(pivotSetPoint);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       shooterSubsystem.setShooterLeftRPM(0);
       shooterSubsystem.setShooterRightRPM(0);
       pivotSubsystem.setSetpoint(RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE);
       feederSubsystem.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (!isFiring && shooterSubsystem.isShooterAtTargetRpm() && pivotSubsystem.pivotReady()) {
            feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED_SHOOT);
            isFiring = true;
            timer.start();
        } 
        if (isFiring && timer.get() > 0.5) {
            return true;
        } else {
            return false;
        }
        
    }
}