package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

public class RabbitDeploySubsystem extends SubsystemBase {
    private final int CURRENT_LIMIT = 10;

    private final CANSparkMax motor = new CANSparkMax(IDs.RABBIT_DEPLOY_DEVICE, MotorType.kBrushless);

    public static final double LOWER_ENDPOINT = -35; 

    public static final double UPPER_ENDPOINT = 0; //Needs to be updated 

    public RabbitDeploySubsystem() {
        motor.setSmartCurrentLimit(CURRENT_LIMIT);
    
        motor.setIdleMode(IdleMode.kBrake);
    
        motor.setInverted(false);

        SmartDashboard.putBoolean("Reset Deploy Encoder", false);
        SmartDashboard.putBoolean("Toggle Deploy Brake", true);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Deploy Encoder", getEncoderPosition());

        if (SmartDashboard.getBoolean("Reset Deploy Encoder", false)) {
            resetEncoder();
            SmartDashboard.putBoolean("Reset Deploy Encoder", false);
        }

        if (SmartDashboard.getBoolean("Reset Deploy Brake", true)) {
            motor.setIdleMode(IdleMode.kBrake);
        } else {
            motor.setIdleMode(IdleMode.kCoast);
        }
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