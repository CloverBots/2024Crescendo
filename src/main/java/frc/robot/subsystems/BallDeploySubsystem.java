package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

public class BallDeploySubsystem extends SubsystemBase{
    private final int CURRENT_LIMIT = 10;

    private final CANSparkMax motor = new CANSparkMax(IDs.BALL_DEPLOY, MotorType.kBrushless);

    public static final double LOWER_ENDPOINT = 0; 

    public static final double UPPER_ENDPOINT = 100; //Needs to be updated 

    public void RabbitDeploySubsystem() {
        motor.setSmartCurrentLimit(CURRENT_LIMIT);
    
        motor.setIdleMode(IdleMode.kBrake);
    
        motor.setInverted(false);
    }

    public void setDeploySpeed(double speed) {
        if ((getEncoderPosition() <= LOWER_ENDPOINT && speed < 0) ||
          (getEncoderPosition() >= UPPER_ENDPOINT && speed > 0)) {
        speed = 0;
        }
        motor.set(speed);
    }

    public double getEncoderPosition() {
        return motor.getEncoder().getPosition(); 
      }

    public void resetEncoder() {
        motor.getEncoder().setPosition(0);
    }
}
