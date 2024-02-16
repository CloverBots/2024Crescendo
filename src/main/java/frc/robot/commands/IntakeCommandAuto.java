// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommandAuto extends Command {

  private static final double INTAKE_SPEED = 0.20;

  private final IntakeSubsystem intakeSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem;

  public IntakeCommandAuto(IntakeSubsystem intakeSubsystem,
      FeederSubsystem feederSubsystem,
      FeederDistanceSensorSubsystem feederDistanceSensorSubsystem
) {

    this.intakeSubsystem = intakeSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.feederDistanceSensorSubsystem = feederDistanceSensorSubsystem;

    addRequirements(intakeSubsystem);
    addRequirements(feederSubsystem);
    addRequirements(feederDistanceSensorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if (!feederDistanceSensorSubsystem.isNoteLoaded()) {
      intakeSubsystem.setIntakeSpeed(INTAKE_SPEED);
      feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeSpeed(0);
    feederSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (feederDistanceSensorSubsystem.isNoteLoaded()) {
      intakeSubsystem.setIntakeSpeed(0);
      feederSubsystem.setSpeed(0);
    }

    return false;
  }
}