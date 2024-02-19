// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

  private static final double INTAKE_SPEED = 1;

  private final IntakeSubsystem intakeSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem;

  private final Supplier<Double> intakeLoadTrigger, intakeEjectTrigger;

  public IntakeCommand(IntakeSubsystem intakeSubsystem,
      FeederSubsystem feederSubsystem,
      FeederDistanceSensorSubsystem feederDistanceSensorSubsystem,
      Supplier<Double> intakeLoadTrigger,
      Supplier<Double> intakeEjectTrigger) {

    this.intakeSubsystem = intakeSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.feederDistanceSensorSubsystem = feederDistanceSensorSubsystem;

    this.intakeLoadTrigger = intakeLoadTrigger;
    this.intakeEjectTrigger = intakeEjectTrigger;

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
    if (intakeLoadTrigger.get() > 0.5 && !feederDistanceSensorSubsystem.isNoteLoaded()) {
      intakeSubsystem.setIntakeSpeed(INTAKE_SPEED);
      feederSubsystem.setSpeed(RobotContainer.FEEDER_SPEED);
    } else if (intakeEjectTrigger.get() > 0.5) {
      intakeSubsystem.setIntakeSpeed(-INTAKE_SPEED);
      feederSubsystem.setSpeed(-RobotContainer.FEEDER_SPEED);
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
      return true;
    } else if (intakeEjectTrigger.get() < 0.5 && intakeSubsystem.isIntakeRunningBackward()) {
      return true;
    } else if (intakeLoadTrigger.get() < 0.5 && intakeSubsystem.isIntakeRunningForward()) {
      return true;
    }

    return false;
  }
}
