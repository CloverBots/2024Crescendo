// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

public class PivotEncoderSubsystem extends SubsystemBase {
  /** Creates a new PivotEncoderSubsystem. */
  private final CANCoder pivotEncoder;
  public final static double PIVOT_LOWER_ENDPOINT = 0;
  public final static double PIVOT_UPPER_ENDPOINT = 1;
    
  public PivotEncoderSubsystem() {
    this.pivotEncoder = new CANCoder(IDs.PIVOT_ENCODER);
    configureCANCoder(pivotEncoder);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getPivotAbsolutePosition() {
    return pivotEncoder.getAbsolutePosition();
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
