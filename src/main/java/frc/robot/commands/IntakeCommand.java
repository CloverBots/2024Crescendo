// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

  private static final double INTAKE_RPM = 20;
  private static final double FEEDER_RPM = 20;

  private final IntakeSubsystem intakeSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem;

  private final Supplier<Boolean> intakeLoadButton, intakeEjectButton;

  public IntakeCommand(IntakeSubsystem intakeSubsystem,
      FeederSubsystem feederSubsystem,
      FeederDistanceSensorSubsystem feederDistanceSensorSubsystem,
      Supplier<Boolean> intakeLoadButton,
      Supplier<Boolean> intakeEjectButton) {

    this.intakeSubsystem = intakeSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.feederDistanceSensorSubsystem = feederDistanceSensorSubsystem;

    this.intakeLoadButton = intakeLoadButton;
    this.intakeEjectButton = intakeEjectButton;

    addRequirements(intakeSubsystem);
    addRequirements(feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeLoadButton.get() && !feederDistanceSensorSubsystem.isNoteLoaded()) {
      intakeSubsystem.setIntakeSpeed(INTAKE_RPM);
      feederSubsystem.setSpeed(FEEDER_RPM);
    } else if (intakeEjectButton.get()) {
      intakeSubsystem.setIntakeSpeed(-INTAKE_RPM);
      feederSubsystem.setSpeed(-FEEDER_RPM);
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
    if (intakeSubsystem.isIntakeRunningForward() && feederDistanceSensorSubsystem.isNoteLoaded()) {
      intakeSubsystem.setIntakeSpeed(0);
      feederSubsystem.setSpeed(0);
    } else if (!intakeEjectButton.get() && intakeSubsystem.isIntakeRunningBackward()) {
      intakeSubsystem.setIntakeSpeed(0);
      feederSubsystem.setSpeed(0);
    }

    return false;
  }
}
