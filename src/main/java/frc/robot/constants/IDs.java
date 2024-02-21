// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.wpilibj.I2C.Port;

/**
 * Contains IDs for the devices on the robot.</p>
 * <strong>To use the IDs for swerve drive components, use {@link SwerveDriveConstants}.</strong>
 */
public final class IDs {
  public static final Port AHRS_PORT_ID = Port.kMXP;

  // Port IDs for the controllers
  public static final int CONTROLLER_DRIVE_PORT = 0;
  public static final int CONTROLLER_OPERATOR_PORT = 1;
  public static final int FEEDER_MOTOR = 30;
  public static final int PIVOT_LEAD_MOTOR = 31; //32
  public static final int PIVOT_FOLLOW_MOTOR = 32; //31
  public static final int PIVOT_ENCODER = 37;

  public static final int INTAKE_MOTOR_ID = 33;
  public static final int SHOOTER_LEFT_MOTOR_ID = 35;
  public static final int SHOOTER_RIGHT_MOTOR_ID = 34;
  public static final int CENTER_MOTOR_1 = 36;
  public static final int CENTER_MOTOR_2 = 46;

  public static final int PIVOT_LIMIT_SWITCH = 0; //digital input port
}