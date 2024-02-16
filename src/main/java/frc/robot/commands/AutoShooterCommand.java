package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionTargetTracker;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederDistanceSensorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShooterCommand extends Command {

    private final static double FEEDER_TIME = 2;
    private final static double MAX_PIVOT_POWER = 0.2;
    private final static double MAX_ANGULAR_ACCELERATION = .5;

    private final ShooterSubsystem shooterSubsystem;
    private final FeederDistanceSensorSubsystem feederDistanceSensorSubsystem;
    private final PivotSubsystem pivotSubsystem;
    private final FireCommand fireCommand;
    private double leftShooterRPM;
    private double rightShooterRPM;
    private double feederSpeed;
    private double pivotAngle;
    private double previousPivotAngle;
    private Timer timer;

    private PIDController pivotController = new PIDController(1.0, 0.0, 0.0); // in degrees
    SlewRateLimiter pivotLimiter;

    public AutoShooterCommand(FeederDistanceSensorSubsystem feederDistanceSensorSubsystem,
            ShooterSubsystem shooterSubsystem,
            PivotSubsystem pivotSubsystem,
            FireCommand fireCommand,
            double leftShooterRPM,
            double rightShooterRPM, double pivotAngle) {

        this.shooterSubsystem = shooterSubsystem;
        this.feederDistanceSensorSubsystem = feederDistanceSensorSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.fireCommand = fireCommand;
        this.leftShooterRPM = leftShooterRPM;
        this.rightShooterRPM = rightShooterRPM;
        this.feederSpeed = feederSpeed;
        this.pivotAngle = pivotAngle;

        timer = new Timer();

        if (pivotAngle > PivotSubsystem.PIVOT_UPPER_ENDPOINT) {
            pivotAngle = PivotSubsystem.PIVOT_UPPER_ENDPOINT;
        }
        if (pivotAngle < PivotSubsystem.PIVOT_LOWER_ENDPOINT) {
            pivotAngle = PivotSubsystem.PIVOT_LOWER_ENDPOINT;
        }

        pivotController.setSetpoint(pivotAngle);
        pivotController.setTolerance(1); // 3 degrees, 0.05 radians
        pivotController.enableContinuousInput(0, 360); // Sets the PID to treat zero and 2 pi as the same value.

        addRequirements(shooterSubsystem);
        addRequirements(feederDistanceSensorSubsystem);
        addRequirements(pivotSubsystem);

        //Used during tuning of shooter
        if (leftShooterRPM == -1) {
            // Used during tuning
            SmartDashboard.putNumber("Shooter right RPM", 0);
            SmartDashboard.putNumber("Shooter left RPM", 0);
            SmartDashboard.putNumber("Shooter angle", 0);
            SmartDashboard.putNumber("Feeder Speed", feederSpeed);
        }
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pivotController.reset();

        previousPivotAngle = pivotAngle;

        double initAngle = pivotSubsystem.getPivotAbsolutePosition();
        if (pivotAngle - initAngle >= 0) {
            pivotLimiter = new SlewRateLimiter(MAX_ANGULAR_ACCELERATION, Integer.MIN_VALUE, 0);
        } else {
            pivotLimiter = new SlewRateLimiter(Integer.MAX_VALUE, -MAX_ANGULAR_ACCELERATION, 0);
        }

        //Used during tuning of shooter
        if (leftShooterRPM == -1) {
            // Used during tuning
            rightShooterRPM = SmartDashboard.getNumber("Shooter right RPM", 0);
            leftShooterRPM = SmartDashboard.getNumber("Shooter left RPM", 0);
            pivotAngle = SmartDashboard.getNumber("Shooter angle", 0);
            feederSpeed = SmartDashboard.getNumber("Feeder Speed", feederSpeed);
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        shooterSubsystem.setShooterLeftRPM(leftShooterRPM);
        shooterSubsystem.setShooterRightRPM(rightShooterRPM);

        // for auto mode,
        // base the pivot angle on the Apriltag distance

        double speed = pivotController.calculate(pivotSubsystem.getPivotAbsolutePosition());
        speed = pivotLimiter.calculate(speed);
        speed = MathUtil.clamp(speed, -MAX_PIVOT_POWER, MAX_PIVOT_POWER);

        pivotSubsystem.setSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShooterLeftRPM(0);
        shooterSubsystem.setShooterRightRPM(0);
        pivotSubsystem.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (fireCommand.isFinished() == true) {
            shooterSubsystem.setShooterLeftRPM(0);
            shooterSubsystem.setShooterRightRPM(0);
            
            //return shooter to parked position
            pivotController.reset();
            pivotAngle = RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE;
            pivotController.setSetpoint(pivotAngle);
        }

        if (pivotAngle == RobotContainer.SHOOTER_PARKED_PIVOT_ANGLE
                && pivotController.atSetpoint()) {
            return true;
        }

        return false;
    }

    private double computePivotAngle(double distance) {
        return 46.7 * distance + 1793; // TO-DO determine proper formula
    }
}