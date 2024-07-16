package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Drives the robot using input from the controller.
 */
public class DriveCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> leftStickX, leftStickY, rightStickX, rightStickY, crawlTrigger, fastRotate;
    private final Supplier<Boolean> yButton, bButton, aButton, xButton, startButton;
    private final Supplier<Integer> dPad;
    private final SlewRateLimiter translationLimiter, turningLimiter;
    private boolean fieldOriented = true;
    public static boolean lockOnMode = false;
    private static int invertJoystick = 1;

    private final double JOYSTICK_DEADZONE = 0.05;

    private PIDController rotationController;

    public DriveCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> leftStickX,
            Supplier<Double> leftStickY,
            Supplier<Double> rightStickX,
            Supplier<Double> rightStickY,
            Supplier<Boolean> yButton,
            Supplier<Boolean> bButton,
            Supplier<Boolean> aButton,
            Supplier<Boolean> xButton,
            Supplier<Boolean> startButton,
            Supplier<Double> crawlTrigger,
            Supplier<Double> fastRotate,
            Supplier<Integer> dPad) {
        this.swerveSubsystem = swerveSubsystem;

        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.rightStickX = rightStickX;
        this.rightStickY = rightStickY;

        this.yButton = yButton;
        this.bButton = bButton;
        this.aButton = aButton;
        this.xButton = xButton;
        this.startButton = startButton;

        this.crawlTrigger = crawlTrigger;
        this.fastRotate = fastRotate;
        this.dPad = dPad;

        this.translationLimiter = new SlewRateLimiter(Constants.DriveConstants.teleOpMaxAccelerationMetersPerSecond);
        this.turningLimiter = new SlewRateLimiter(Constants.DriveConstants.teleOpMaxAngularAccelerationUnitsPerSecond);

        // using degrees
        this.rotationController = new PIDController(5.0, .3, 0); // 0.017, 0, 0
        this.rotationController.enableContinuousInput(0, 2 * Math.PI);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // Start calculating our speeds
        ChassisSpeeds speeds = calculateSpeeds();

        // Handle all the toggle buttons on the joystick
        handleToggleButtons();

        SmartDashboard.putBoolean("Field Oriented", fieldOriented);

        // Feed the calculated speeds to the swerve subsystem
        swerveSubsystem.setSpeed(speeds, fieldOriented);
    }

    /**
     * Correctly handles all the toggle buttons on the controller (field oriented,
     * point mode, etc.)
     */
    private void handleToggleButtons() {
        if (startButton.get() == true) {
            swerveSubsystem.resetPose(null);
        }
    }

    /**
     * Calculates the translation (X/Y) speeds and turning speeds from the
     * controller, depending on the current mode.
     * 
     * @return
     */
    private ChassisSpeeds calculateSpeeds() {
        double rotationSpeed;

        // Calculate the Translation speeds, invert joystick for robot relative
        double[] xySpeeds = calculateTranslationSpeeds(invertJoystick * leftStickX.get(),
                invertJoystick * leftStickY.get());

        rotationSpeed = calculateTurningSpeeds(rightStickX.get());

        return new ChassisSpeeds(xySpeeds[1], xySpeeds[0], rotationSpeed);
    }

    private double calculateTurningSpeeds(double turningSpeed) {

        // Apply a deadzone. This will prevent the robot from moving at very small
        // values
        turningSpeed = Math.abs(turningSpeed) > JOYSTICK_DEADZONE ? turningSpeed : 0.0;

        double turningSpeedMultiplier; // The max speed of the robot if the stick is pressed all the way.

        // Slows both rotation and translation if the crawl trigger is pressed.
        if (crawlTrigger.get() >= 0.5) {
            turningSpeedMultiplier = Constants.DriveConstants.teleOpSlowAngularSpeed;
        } // Fast rotate if crawl is not pressed and fast rotate is
        else if (fastRotate.get() >= 0.5 && crawlTrigger.get() <= 0.5) {
            turningSpeedMultiplier = Constants.DriveConstants.teleOpMaxAngularSpeed;
        } else {
            turningSpeedMultiplier = Constants.DriveConstants.teleOpNormalAngularSpeed;
        }
        turningSpeed = turningLimiter.calculate(turningSpeed);
        turningSpeed *= turningSpeedMultiplier;
        return turningSpeed;
    }

    /**
     * Calculates the Translation (X and Y) speeds of the robot from the controller
     * joystick.
     * 
     * @param xSpeed The raw X value of the joystick.
     * @param ySpeed The raw Y value of the joystick.
     * @return A double[] array of length 2, containing the calculated X and Y
     *         speeds respectively.
     */
    private double[] calculateTranslationSpeeds(double xSpeed, double ySpeed) {
        double[] xySpeeds = new double[2]; // Will contain the calculated X and Y speeds

        double speedMultiplier; // The maximum speed of the robot in any direction if the joystick is pressed
                                // all the way

        // Max speed decresed if crawl trigger is pressed
        if (crawlTrigger.get() >= 0.5)
            speedMultiplier = Constants.DriveConstants.TELEOP_SLOW_SPEED_METERS_PER_SECOND;

        // Use default max speed if it is not pressed
        else
            speedMultiplier = Constants.DriveConstants.TELEOP_MAX_SPEED_METERS_PER_SECOND;

        /*
         * This is the Magnitude. It is the hypotenuse of the X and Y speeds.
         * It can be thought of as the distance the joystick is from the center when it
         * is pressed.
         * If you press the joystick all the way forward, it will always be 1.0 no
         * matter what direction it is.
         * This is used to make sure that speed calculations, such as scaling and the
         * deadzones, are always consistant regardless of the direction the robot is
         * travelling in.
         */
        double magnitude = Math.hypot(ySpeed, xSpeed);

        // Apply the deadzone. This will prevent the robot from moving at very small
        // values
        magnitude = Math.abs(magnitude) > JOYSTICK_DEADZONE ? magnitude : 0.0;

        // Most controllers are not completely circular (meaning the magnitude can go
        // above 1.0 in certain directions), so we cap it at 1.0.
        magnitude = Math.min(magnitude, 1);

        // Apply a power curve. This makes it so that the robot will move slower at
        // lower inputs compared to if it weren't. This makes small movements easier.
        magnitude = Math.pow(magnitude, 3);

        // Limits the acceleration for translation
        magnitude = translationLimiter.calculate(magnitude);

        magnitude *= speedMultiplier; // Scales the magnitude

        // Multiply the raw X and Y inputs by the magnitude to get the correct X and Y
        // translation speeds
        xSpeed *= magnitude;
        ySpeed *= magnitude;

        xySpeeds[0] = xSpeed;
        xySpeeds[1] = ySpeed;

        return xySpeeds;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules(); // Stop all swerve modules if the command ends.
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}