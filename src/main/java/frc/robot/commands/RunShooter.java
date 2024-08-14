// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooter extends Command {
    public final ShooterSubsystem shooterSubsystem;
    public final PivotSubsystem pivotSubsystem;
    public final FeederSubsystem feederSubsystem;
    public final int leftShooterRPM;
    public final int rightShooterRPM;
    public final double pivotSetPoint;
    public Timer timer = new Timer();
    public boolean isFiring = false;

    public RunShooter(ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, FeederSubsystem feederSubsystem,
            int leftShooterRPM, int rightShooterRPM, double pivotSetPoint) {
        this.shooterSubsystem = shooterSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.leftShooterRPM = leftShooterRPM;
        this.rightShooterRPM = rightShooterRPM;
        this.pivotSetPoint = pivotSetPoint;

        addRequirements(shooterSubsystem);
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterLeftRPM(leftShooterRPM);
        shooterSubsystem.setShooterRightRPM(rightShooterRPM);
        pivotSubsystem.setPivotControllerSetpoint(pivotSetPoint);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShooterLeftRPM(0);
        shooterSubsystem.setShooterRightRPM(0);
        pivotSubsystem.setSetpoint(PivotConstants.SHOOTER_PARKED_PIVOT_ANGLE);
        feederSubsystem.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        if (!isFiring && shooterSubsystem.isShooterAtTargetRpm() && pivotSubsystem.pivotReady()) {
            feederSubsystem.setSpeed(IntakeConstants.FEEDER_SPEED_SHOOT);
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