package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

public class BallShooterSubsystem extends SubsystemBase{
    private final int CURRENT_LIMIT = 10;

    private final CANSparkMax motor = new CANSparkMax(IDs.SHOOTER_DEVICE, MotorType.kBrushless);

    /** Creates a new FeederSubsystem. */
    public BallShooterSubsystem() {

        motor.setInverted(true);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public double getEncoderPosition() {
        return motor.getEncoder().getPosition(); 
      }

    public void resetEncoder() {
        motor.getEncoder().setPosition(0);
    }
}
