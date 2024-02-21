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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.VisionConfiguration;
import frc.robot.VisionTargetTracker;
import frc.robot.constants.IDs;

public class PivotSubsystem extends SubsystemBase {
  private final CANSparkMax pivotLeadMotor;
  private final CANSparkMax pivotFollowMotor;

  private final CANCoder pivotEncoder;
  public final static double PIVOT_LOWER_ENDPOINT = 1;
  public final static double PIVOT_UPPER_ENDPOINT = 100;

  private final int CURRENT_LIMIT = 100;

  private final DigitalInput limitSwitch = new DigitalInput(IDs.PIVOT_LIMIT_SWITCH);

  public PivotSubsystem() {
    this.pivotLeadMotor = new CANSparkMax(IDs.PIVOT_LEAD_MOTOR, MotorType.kBrushless);
    this.pivotFollowMotor = new CANSparkMax(IDs.PIVOT_FOLLOW_MOTOR, MotorType.kBrushless);

    pivotFollowMotor.follow(pivotLeadMotor, true);

    pivotLeadMotor.setInverted(false);

    pivotLeadMotor.setIdleMode(IdleMode.kBrake);
    pivotFollowMotor.setIdleMode(IdleMode.kBrake);

    pivotLeadMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    pivotFollowMotor.setSmartCurrentLimit(CURRENT_LIMIT);

    this.pivotEncoder = new CANCoder(IDs.PIVOT_ENCODER);
    configureCANCoder(pivotEncoder);
  }

  public void resetPivotEncoder() {
    pivotLeadMotor.getEncoder().setPosition(getPivotAbsolutePosition());
  }

  public void setSpeed(double speed) {
    pivotLeadMotor.set(speed);
  }

  public void stop() {
    pivotLeadMotor.set(0);
  }

  @Override
  public void periodic() {
    if (getPivotAbsolutePosition() < PIVOT_LOWER_ENDPOINT
        || getPivotAbsolutePosition() > PIVOT_UPPER_ENDPOINT) {
      System.out.println("PIVOT ENDPOINT reached!!!");
      pivotLeadMotor.set(0);
    }

    if (!limitSwitch.get()) {
      System.out.println("PIVOT LIMIT SWITCH reached!!!");
      pivotLeadMotor.set(0);
    }

  }

  public double getPivotAbsolutePosition() {
    SmartDashboard.putNumber("Absolut encoder", pivotEncoder.getAbsolutePosition());
    return pivotEncoder.getAbsolutePosition();
  }

  private void configureCANCoder(CANCoder cancoder) {
    CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
    encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    encoderConfig.magnetOffsetDegrees = -202.84; // Need Offset
    encoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    cancoder.configAllSettings(encoderConfig);
  }

}