package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

public class RabbitIntakeSubsystem extends SubsystemBase {
    private final double INTAKE_DEFAULT_SPEED = 1.0; //Increase after testing
    
    private final TalonSRX intakeMotor = new TalonSRX(IDs.TALONSRX_MOTOR);
    private double speed;

    public void periodic() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void startIntake(double speed) {
        this.speed = speed;
    }

    public void startIntake() {
        startIntake(INTAKE_DEFAULT_SPEED);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void stop() {
        speed = 0;
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
    

    
    

}
