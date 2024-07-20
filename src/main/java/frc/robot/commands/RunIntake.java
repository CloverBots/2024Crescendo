// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends Command {
  /** Creates a new RunIntake. */
  public final IntakeSubsystem intake;
  public final double intakeMotorSpeed;
  public final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem;
  

  public RunIntake(IntakeSubsystem intake, double intakeMotorSpeed, FeederDistanceSensorSubsystem feederDistanceSensorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.intakeMotorSpeed = intakeMotorSpeed;
    this.feederDistanceSensorSubsystem = feederDistanceSensorSubsystem;

    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeSpeed(1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (feederDistanceSensorSubsystem.isNoteLoaded()) {
        return true;
    } else {
        return false;
    }
  }
}