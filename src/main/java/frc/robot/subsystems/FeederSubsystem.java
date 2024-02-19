// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

public class FeederSubsystem extends SubsystemBase {

private final CANSparkMax feederMotor;

private final int CURRENT_LIMIT = 100;

  public FeederSubsystem() {
    this.feederMotor = new CANSparkMax(IDs.FEEDER_MOTOR, MotorType.kBrushless);

    feederMotor.setInverted(false);
    feederMotor.setIdleMode(IdleMode.kCoast);
    feederMotor.setInverted(true);
    feederMotor.setSmartCurrentLimit(CURRENT_LIMIT);
  }

  public void setSpeed(double speed) {
    feederMotor.set(speed);
  }
}