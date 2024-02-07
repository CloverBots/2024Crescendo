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
import frc.robot.constants.IDs;

public class PivotSubsystem extends SubsystemBase {
  private final CANSparkMax pivotLeadMotor;
  private final CANSparkMax pivotFollowMotor;
  private final CANCoder pivotEncoder;
  private PIDController pivotPIDController;

  public PivotSubsystem() {
    this.pivotLeadMotor = new CANSparkMax(IDs.PIVOT_LEAD_MOTOR, MotorType.kBrushless);
    this.pivotFollowMotor = new CANSparkMax(IDs.PIVOT_FOLLOW_MOTOR, MotorType.kBrushless);
    this.pivotEncoder = new CANCoder(IDs.PIVOT_ENCODER);
    configureCANCoder(pivotEncoder);

    pivotFollowMotor.follow(pivotLeadMotor);

    pivotLeadMotor.setInverted(false);
    pivotFollowMotor.setInverted(false);
    pivotLeadMotor.setIdleMode(IdleMode.kBrake);
    pivotFollowMotor.setIdleMode(IdleMode.kBrake);
  }

  public double getPivotAbsolutePosition() {
    return pivotEncoder.getAbsolutePosition();
  }

  public double getPivotPosition() {
    double pos = pivotLeadMotor.getEncoder().getPosition();
    boolean sign = pos < 0;
    pos = Math.abs(pos);
    pos = pos % (2*Math.PI);
    if (sign) pos = (2*Math.PI) - pos;
    return pos;
  }

  public void resetPivotEncoder() {
    pivotLeadMotor.getEncoder().setPosition(getPivotAbsolutePosition());
  }

  public void setPosition(){
    pivotFollowMotor.set(pivotPIDController.calculate(getPivotPosition()));
  }

  public void stop(){
    pivotFollowMotor.set(0);
  }

  private void configureCANCoder(CANCoder cancoder) {
    CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
    encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    encoderConfig.magnetOffsetDegrees = 0; // Need Offset
    encoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    cancoder.configAllSettings(encoderConfig);
  }

}