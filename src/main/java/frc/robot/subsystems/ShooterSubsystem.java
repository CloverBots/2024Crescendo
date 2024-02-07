package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class ShooterSubsystem extends SubsystemBase {
    private static final double WHEEL_DIAMETER_METERS = 0.1524;
    private static final double ENCODER_POSITION_CONVERSION_FACTOR = 1;// 0.1 * WHEEL_DIAMETER_METERS * Math.PI;
    private static final double ENCODER_VELOCITY_CONVERSION_FACTOR = 1;// ENCODER_POSITION_CONVERSION_FACTOR * 60.0;
    private static final double MAX_RPM = 5700;

    private static final double SHOOTER_P = 0.0; // 8e-5;
    private static final double SHOOTER_I = 0.0;
    private static final double SHOOTER_D = 0.0;
    private static final double SHOOTER_Iz = 0;
    private static final double SHOOTER_FF = 0.0005;// 0.00017;
    private static final double SHOOTER_MAX_OUTPUT = 1;
    private static final double SHOOTER_MIN_OUTPUT = -1;

    private CANSparkMax motorLeft = new CANSparkMax(IDs.SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);

    private CANSparkMax motorRight = new CANSparkMax(IDs.SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);

    // private final MotorControllerGroup shooterMotors = new
    // MotorControllerGroup(shooterLeadMotor, shooterFollowMotor1);

    private final RelativeEncoder encoderLeft = motorLeft.getEncoder();

    private final RelativeEncoder encoderRight = motorRight.getEncoder();

    private SparkPIDController pidControllerLeft;

    private SparkPIDController pidControllerRight;

    /** Creates a new Shooter. */
    public ShooterSubsystem() {
        // shooterFollowMotor1.setInverted(true);

        configureEncoder(encoderLeft);

        configureEncoder(encoderRight);

        pidControllerLeft = motorLeft.getPIDController();

        pidControllerRight = motorRight.getPIDController();

        // set PID coefficients
        pidControllerLeft.setP(SHOOTER_P);
        pidControllerLeft.setI(SHOOTER_I);
        pidControllerLeft.setD(SHOOTER_D);
        pidControllerLeft.setIZone(SHOOTER_Iz);
        pidControllerLeft.setFF(SHOOTER_FF);
        pidControllerLeft.setOutputRange(SHOOTER_MIN_OUTPUT, SHOOTER_MAX_OUTPUT);

        pidControllerRight.setP(SHOOTER_P);
        pidControllerRight.setI(SHOOTER_I);
        pidControllerRight.setD(SHOOTER_D);
        pidControllerRight.setIZone(SHOOTER_Iz);
        pidControllerRight.setFF(SHOOTER_FF);
        pidControllerRight.setOutputRange(SHOOTER_MIN_OUTPUT, SHOOTER_MAX_OUTPUT);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void stop() {
        pidControllerLeft.setReference(0, CANSparkMax.ControlType.kVelocity);

        pidControllerRight.setReference(0, CANSparkMax.ControlType.kVelocity);
    }

    private void configureEncoder(RelativeEncoder encoder) {
        encoder.setPositionConversionFactor(ENCODER_POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(ENCODER_VELOCITY_CONVERSION_FACTOR);
    }

    public void setShooterLeftRPM(double rpm) {

        /**
         * PIDController objects are commanded to a set point using the
         * SetReference() method.
         * 
         * The first parameter is the value of the set point, whose units vary
         * depending on the control type set in the second parameter.
         * 
         * The second parameter is the control type can be set to one of four
         * parameters:
         * com.revrobotics.CANSparkMax.ControlType.kDutyCycle
         * com.revrobotics.CANSparkMax.ControlType.kPosition
         * com.revrobotics.CANSparkMax.ControlType.kVelocity
         * com.revrobotics.CANSparkMax.ControlType.kVoltage
         */
        if (rpm > MAX_RPM) {
            rpm = MAX_RPM;
        }

        pidControllerLeft.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    public void setShooterRightRPM(double rpm) {
        if (rpm > MAX_RPM) {
            rpm = MAX_RPM;
        }
        
        pidControllerRight.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

}