// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    public static CANifier canifier = new CANifier(2);

    /** Creates a new LEDSubsystem. */
    public LEDSubsystem() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setLEDGreen() {
        canifier.setLEDOutput(100, CANifier.LEDChannel.LEDChannelA); // GREEN
        canifier.setLEDOutput(0, CANifier.LEDChannel.LEDChannelB);
        canifier.setLEDOutput(0, CANifier.LEDChannel.LEDChannelC);
    }

    public void setLEDRed() {
        canifier.setLEDOutput(100, CANifier.LEDChannel.LEDChannelB); // RED
        canifier.setLEDOutput(0, CANifier.LEDChannel.LEDChannelA);
        canifier.setLEDOutput(0, CANifier.LEDChannel.LEDChannelC);
    }

    public void setLEDBlue() {
        canifier.setLEDOutput(100, CANifier.LEDChannel.LEDChannelC); // BLUE
        canifier.setLEDOutput(0, CANifier.LEDChannel.LEDChannelB);
        canifier.setLEDOutput(0, CANifier.LEDChannel.LEDChannelA);
    }

}
