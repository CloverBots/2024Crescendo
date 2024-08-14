// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class AutoIntakeCommand extends Command {
    public final IntakeSubsystem intake;
    public final double intakeMotorSpeed;
    public final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem;
    public final FeederSubsystem feederSubsystem;
    public final PivotSubsystem pivotSubsystem;

    public AutoIntakeCommand(IntakeSubsystem intake, double intakeMotorSpeed,
            FeederDistanceSensorSubsystem feederDistanceSensorSubsystem, FeederSubsystem feederSubsystem, PivotSubsystem pivotSubsystem) {
        this.intake = intake;
        this.intakeMotorSpeed = intakeMotorSpeed;
        this.feederDistanceSensorSubsystem = feederDistanceSensorSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        pivotSubsystem.setPivotControllerSetpoint(PivotConstants.PIVOT_PARKED_ANGLE);
        intake.setIntakeSpeed(intakeMotorSpeed);
        feederSubsystem.setSpeed(IntakeConstants.FEEDER_SPEED_INTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0);
        feederSubsystem.setSpeed(0);
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