// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class PivotSubsystem extends PIDSubsystem {
    private final CANSparkMax pivotLeadMotor;
    private final CANSparkMax pivotFollowMotor;
    private final CANcoder pivotEncoder;
    private final DigitalInput limitSwitch = new DigitalInput(PivotConstants.PIVOT_LIMIT_SWITCH);

    double speed = 0;

    public PivotSubsystem() {
        super(new PIDController(0.02, 0.0045, 0));
        getController().setTolerance(0.5);
        getController().enableContinuousInput(0, 360); // Sets the PID to treat zero and 2 pi as the same value.
        disable(); // start with PID disabled

        pivotLeadMotor = new CANSparkMax(PivotConstants.PIVOT_LEAD_MOTOR, MotorType.kBrushless);
        pivotFollowMotor = new CANSparkMax(PivotConstants.PIVOT_FOLLOW_MOTOR, MotorType.kBrushless);

        pivotFollowMotor.follow(pivotLeadMotor, true);

        pivotLeadMotor.setInverted(true);

        pivotLeadMotor.setIdleMode(IdleMode.kBrake);
        pivotFollowMotor.setIdleMode(IdleMode.kBrake);

        pivotLeadMotor.setSmartCurrentLimit(Constants.CURRENT_LIMIT);
        pivotFollowMotor.setSmartCurrentLimit(Constants.CURRENT_LIMIT);

        this.pivotEncoder = new CANcoder(PivotConstants.PIVOT_ENCODER);
    }

    @Override
    public double getMeasurement() {
        return getPivotAbsolutePosition();
    }

    @Override
    public void useOutput(double output, double setpoint) {

        output = MathUtil.clamp(output, -PivotConstants.MAX_PIVOT_POWER_PID, PivotConstants.MAX_PIVOT_POWER_PID);
        speed = output;
        if (pivotLeadMotor != null) {
            pivotLeadMotor.set(speed);
        }
    }

    public void resetPivotEncoder() {
        pivotLeadMotor.getEncoder().setPosition(getPivotAbsolutePosition());
    }

    public void setSpeed(double speed) {
        disable();
        pivotLeadMotor.set(speed);
        this.speed = speed;
    }

    public void stop() {
        pivotLeadMotor.set(0);
        disable();
    }

    public void setPivotControllerSetpoint(double angle) {
        getController().reset();
        setSetpoint(angle);
        enable();
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }

    public boolean pivotReady() {
        if (Math.abs(getPivotAbsolutePosition() - getSetpoint()) < 2) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {

        super.periodic(); // super controlls calling PID stuff

        if ((getPivotAbsolutePosition() < PivotConstants.PIVOT_PHYSICAL_LOWER_ENDPOINT &&
                speed < 0) || (getPivotAbsolutePosition() > PivotConstants.PIVOT_UPPER_ENDPOINT && speed > 0)) {
            System.out.println("PIVOT ENDPOINT reached!!!");
            speed = 0;
            disable();
        }

        if (!limitSwitch.get()) {
            if (speed < 0) {
                System.out.println("Limit Switch reached!!!"); // Limit switch is at 3.5
                speed = 0;
                disable();
            }
        }

        pivotLeadMotor.set(speed);
    }

    public double getPivotAbsolutePosition() {
        SmartDashboard.putNumber("Absolute encoder", pivotEncoder.getAbsolutePosition().getValueAsDouble() * 360);
        return pivotEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }
}