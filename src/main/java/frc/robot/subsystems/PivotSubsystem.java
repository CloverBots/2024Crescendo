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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.constants.IDs;

public class PivotSubsystem extends PIDSubsystem {

    private final static double MAX_PIVOT_POWER_PID = 0.6; 

    private final CANSparkMax pivotLeadMotor;
    private final CANSparkMax pivotFollowMotor;

    private final CANCoder pivotEncoder;
    public final static double PIVOT_LOWER_ENDPOINT = 2;
    public final static double PIVOT_UPPER_ENDPOINT = 100;

    private final int CURRENT_LIMIT = 100;

    private final DigitalInput limitSwitch = new DigitalInput(IDs.PIVOT_LIMIT_SWITCH);

    // private PIDController pivotController = new PIDController(0.025, 0.0, 0); //
    // in degrees

    double speed = 0;

    public PivotSubsystem() {

        super(new PIDController(0.01, 0.0025, 0)); // P=0.025, I=0, D = 0
        getController().setTolerance(0.5);
        getController().enableContinuousInput(0, 360); // Sets the PID to treat zero and 2 pi as the same value.
        disable(); // start with PID disabled

        pivotLeadMotor = new CANSparkMax(IDs.PIVOT_LEAD_MOTOR, MotorType.kBrushless);
        pivotFollowMotor = new CANSparkMax(IDs.PIVOT_FOLLOW_MOTOR, MotorType.kBrushless);

        pivotFollowMotor.follow(pivotLeadMotor, true);

        pivotLeadMotor.setInverted(true);

        pivotLeadMotor.setIdleMode(IdleMode.kBrake);
        pivotFollowMotor.setIdleMode(IdleMode.kBrake);

        pivotLeadMotor.setSmartCurrentLimit(CURRENT_LIMIT);
        pivotFollowMotor.setSmartCurrentLimit(CURRENT_LIMIT);

        this.pivotEncoder = new CANCoder(IDs.PIVOT_ENCODER);

        configureCANCoder(pivotEncoder);
    }

    @Override
    public double getMeasurement() {
        return getPivotAbsolutePosition();
    }

    @Override
    public void useOutput(double output, double setpoint) {
        
        output = MathUtil.clamp(output, -MAX_PIVOT_POWER_PID, MAX_PIVOT_POWER_PID);
        speed = output;
        // checkLimits();
        // pivotLeadMotor is null as robot first starts up and this method gets called
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
        if (Math.abs(getPivotAbsolutePosition() - getSetpoint()) < 1) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {

        super.periodic(); // super controlls calling PID stuff

        if ((getPivotAbsolutePosition() < PIVOT_LOWER_ENDPOINT &&
                speed < 0) || (getPivotAbsolutePosition() > PIVOT_UPPER_ENDPOINT && speed > 0)) {
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
        SmartDashboard.putNumber("Absolute encoder", pivotEncoder.getAbsolutePosition());
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