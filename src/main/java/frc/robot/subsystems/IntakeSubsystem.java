package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

public class IntakeSubsystem extends SubsystemBase {
    private final int CURRENT_LIMIT = 10;

    private final CANSparkMax motor = new CANSparkMax(IDs.INTAKE_MOTOR_ID, MotorType.kBrushless);

    public IntakeSubsystem() {
        motor.setSmartCurrentLimit(CURRENT_LIMIT);

        motor.setIdleMode(IdleMode.kCoast);

        motor.setInverted(false);
    }

    public void setIntakeSpeed(double speed) {
        motor.set(speed);
    }

   
}