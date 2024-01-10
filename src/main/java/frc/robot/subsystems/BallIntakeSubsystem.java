package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

public class BallIntakeSubsystem extends SubsystemBase {
    private final int CURRENT_LIMIT = 10;

    private final CANSparkMax motor = new CANSparkMax(IDs.INTAKE_DEVICE, MotorType.kBrushless);

    public BallIntakeSubsystem() {
        motor.setSmartCurrentLimit(CURRENT_LIMIT);

        motor.setIdleMode(IdleMode.kBrake);

        motor.setInverted(false);
    }

    public void setIntakeSpeed(double speed) {
        motor.set(speed);
    }

    public double getIntakeEncoderPosition() {
        return motor.getEncoder().getPosition(); 
    }

    public void setIntakeMaximumPosition(double min, double max) {
        motor.setSoftLimit(SoftLimitDirection.kForward, (float) max);
        motor.setSoftLimit(SoftLimitDirection.kForward, (float) min);
    }

    public void resetEncoder() {
        motor.getEncoder().setPosition(0);
    }
    
}
