package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

public class IntakeSubsystem extends SubsystemBase {
    private final int CURRENT_LIMIT = 10;

    private final CANSparkMax motor = new CANSparkMax(IDs.INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final TalonSRX centerMotor1 = new TalonSRX(IDs.CENTER_MOTOR_1);
    private final TalonSRX centerMotor2 = new TalonSRX(IDs.CENTER_MOTOR_2);

    public IntakeSubsystem() {
        motor.setSmartCurrentLimit(CURRENT_LIMIT);

        motor.setIdleMode(IdleMode.kCoast);

        motor.setInverted(false);
    }

    public void setIntakeSpeed(double speed) {
        motor.set(speed);
        centerMotor1.set(TalonSRXControlMode.PercentOutput, speed / 2);
        centerMotor2.set(TalonSRXControlMode.PercentOutput, -speed / 2);
    }

    public boolean isIntakeRunningForward() {
        if (motor.get() > 0) {
            return true;
        } else {
            return false;
        }
    }
   
    public boolean isIntakeRunningBackward() {
        if (motor.get() < 0) {
            return true;
        } else {
            return false;
        }
    }
}