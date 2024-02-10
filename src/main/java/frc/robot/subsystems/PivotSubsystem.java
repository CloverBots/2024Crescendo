// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.IDs;
import frc.robot.subsystems.PivotEncoderSubsystem;

public class PivotSubsystem extends SubsystemBase {
  private final CANSparkMax pivotLeadMotor;
  private final CANSparkMax pivotFollowMotor;

  PivotEncoderSubsystem pivotEncoderSubsystem = new PivotEncoderSubsystem();

  private final int CURRENT_LIMIT = 10;
  private String owner = "";

  public PivotSubsystem() {
    this.pivotLeadMotor = new CANSparkMax(IDs.PIVOT_LEAD_MOTOR, MotorType.kBrushless);
    this.pivotFollowMotor = new CANSparkMax(IDs.PIVOT_FOLLOW_MOTOR, MotorType.kBrushless);

    pivotFollowMotor.follow(pivotLeadMotor);

    pivotLeadMotor.setInverted(false);
    pivotFollowMotor.setInverted(false);
    pivotLeadMotor.setIdleMode(IdleMode.kBrake);
    pivotFollowMotor.setIdleMode(IdleMode.kBrake);

    pivotLeadMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    pivotFollowMotor.setSmartCurrentLimit(CURRENT_LIMIT);
  }

  public void resetPivotEncoder() {
    pivotLeadMotor.getEncoder().setPosition(pivotEncoderSubsystem.getPivotAbsolutePosition());
  }

  public void setSpeed(double speed){
    pivotLeadMotor.set(speed);
  }

  public void stop() {
    pivotLeadMotor.set(0);
  }

  public String getOwner() {
    return owner;
  }

  public void setOwner(String owner) {
    this.owner = owner;
  }
}