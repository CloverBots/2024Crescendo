// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package modulelib;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.SwervePID;

/** Add your docs here. */
public class SwerveModule {
    private CANSparkMax turnMotor;
    private TalonFX driveMotor;
    private PIDController pidController;
    private CANcoder encoder;
    private double drivePosition;
    private double driveVelocity;
    private double maxVelocity;
    private double maxVoltage = 60;

    public SwerveModule(int turnMotorId, int driveMotorId, int encoderId, boolean driveInverted,
            double maxVelocity) {
        this.turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
        this.driveMotor = new TalonFX(driveMotorId);
        this.pidController = new PIDController(SwervePID.p, SwervePID.i, SwervePID.d);
        this.encoder = new CANcoder(encoderId);
        this.maxVelocity = maxVelocity;

        this.pidController.enableContinuousInput(-180, 180);

        driveMotor = new TalonFX(driveMotorId);
        driveMotor.getConfigurator().setPosition(0.0);

        this.driveMotor.setInverted(driveInverted);
    }

    public SwerveModule(SwerveModuleConfig config, double maxVelocity, double maxVoltage) {
        this(config.angleMotorId,
                config.driveMotorId,
                config.encoderId,
                config.drive_inverted,
                maxVelocity);

        turnMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
    }

    private void drive(double speedMetersPerSecond, double angle) {
        double voltage = (speedMetersPerSecond / maxVelocity) * maxVoltage;
        driveMotor.setVoltage(voltage);
        turnMotor.setVoltage(-pidController.calculate(this.getEncoder(), angle));
    }

    public void drive(SwerveModuleState state) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, new Rotation2d(getEncoderRadians()));
        this.drive(optimized.speedMetersPerSecond, optimized.angle.getDegrees());
    }

    public double getEncoder() {
        return encoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }

    private Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getEncoder());
    }

    public double getEncoderRadians() {
        return Units.degreesToRadians(getEncoder());
    }

    public SwerveModulePosition getPosition() {
        drivePosition = driveMotor.getPosition().getValueAsDouble() / Constants.DriveConstants.DRIVE_GEAR_RATIO;
        return new SwerveModulePosition(
                drivePosition * Constants.DriveConstants.WHEEL_CIRCUMFERENCE,
                getRotation());
    }

    public SwerveModuleState getState() {
        driveVelocity = driveMotor.getVelocity().getValueAsDouble() / Constants.DriveConstants.DRIVE_GEAR_RATIO;
        return new SwerveModuleState(driveVelocity * Constants.DriveConstants.WHEEL_CIRCUMFERENCE,
                getRotation());
    }
}