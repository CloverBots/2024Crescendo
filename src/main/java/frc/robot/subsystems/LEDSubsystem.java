// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDs.HSVtoRGB;
import frc.robot.subsystems.LEDs.MovingAverage;

public class LEDSubsystem extends SubsystemBase {
    public static CANifier canifier = new CANifier(2);

    private static LEDSubsystem instance = null;

    public static LEDSubsystem getInstance() {
        if (instance == null)
            instance = new LEDSubsystem();
        return instance;
    }

    boolean lit = false;
    double lastOnTime = 0.0;
    double lastOffTime = 0.0;
    double transTime = 0.0;

    public enum State {
        OFF(0.0, 0.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        SOLID_RED(255.0, 0.0, 0.0, Double.POSITIVE_INFINITY, 0, false),
        SOLID_ORANGE(255.0, 50.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        SOLID_YELLOW(255.0, 255.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        SOLID_GREEN(0.0, 255.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        SOLID_BLUE(0.0, 0.0, 255.0, Double.POSITIVE_INFINITY, 0.0, false),
        SOLID_PURPLE(255.0, 0.0, 255.0, Double.POSITIVE_INFINITY, 0.0, false),
        SOLID_PINK(255.0, 20.0, 30.0, Double.POSITIVE_INFINITY, 0.0, false),
        FLASHING_RED(255.0, 0.0, 0.0, 0.125, 0.125, false),
        FLASHING_ORANGE(255.0, 165.0, 0.0, 0.125, 0.125, false),
        FLASHING_YELLOW(255.0, 255.0, 0.0, 0.125, 0.125, false),
        FLASHING_GREEN(0.0, 255.0, 0.0, 0.125, 0.125, false),
        FLASHING_BLUE(0.0, 0.0, 255.0, 0.125, 0.125, false),
        FLASHING_PURPLE(255.0, 0.0, 255.0, 0.125, 0.125, false),
        FLASHING_PINK(255.0, 20.0, 30.0, 0.125, 0.125, false),
        RAINBOW(0, true),
        BREATHING_GREEN(120, 10.0, true),
        BREATHING_BLUE(150, 5.0, true);

        double red, green, blue, onTime, offTime, cycleTime, transitionTime;
        float startingHue;
        List<List<Double>> colors = new ArrayList<List<Double>>();
        boolean isCycleColors;

        private State(double r, double g, double b, double onTime, double offTime, boolean isCycleColors) {
            red = r / 255.0;
            green = g / 255.0;
            blue = b / 255.0;
            this.onTime = onTime;
            this.offTime = offTime;
        }

        private State(float hue, boolean cycle) {
            this.startingHue = hue;
            this.isCycleColors = cycle;
        }

        private State(float hue, double transTime, boolean cycle) {
            this.startingHue = hue;
            this.transitionTime = transTime;
            this.isCycleColors = cycle;
        }

        private State(List<List<Double>> colors, double cycleTime, boolean isCycleColors, double transitionTime) {
            this.colors = colors;
            this.cycleTime = cycleTime;
            this.isCycleColors = isCycleColors;
            this.transitionTime = transitionTime;
        }
    }

    private State currentState = State.OFF;

    public State getState() {
        return currentState;
    }

    private void setState(State newState) {
        if (newState != currentState) {
            currentState = newState;
            lastOffTime = 0.0;
            lastOnTime = 0.0;
            lit = false;
        }
    }

    public void setLEDs(double r, double g, double b) {
        // A: Green
        // B: Red
        // C: Blue
        canifier.setLEDOutput(r, LEDChannel.LEDChannelB);
        canifier.setLEDOutput(b, LEDChannel.LEDChannelC);
        canifier.setLEDOutput(g, LEDChannel.LEDChannelA);
    }

    public void conformToState(State state) {
        setState(state);
    }

    public double stateHue = State.RAINBOW.startingHue;
    public float saturation = 1.0f; // Ensures that the colors are on the outside of the color wheel
    public float value = 0.5f; // Hardcoded brightness
    public double startingTransTime = 0.0;
    public boolean resetBreath = false;

    @Override
    public void periodic() {
        outputTelemetry();
        checkSystem();

        double timestamp = Timer.getFPGATimestamp();

        if (currentState == State.RAINBOW && currentState.isCycleColors == true) {
            stateHue += 2;
            if (stateHue >= (360 - State.RAINBOW.startingHue)) {
                stateHue = State.RAINBOW.startingHue;
            }

            float rgb[] = new float[3];
            MovingAverage averageR = new MovingAverage(5);
            MovingAverage averageG = new MovingAverage(5);
            MovingAverage averageB = new MovingAverage(5);

            if (saturation > 1) {
                saturation = 1;
            }
            if (saturation < 0) {
                saturation = 0;
            }
            if (value > 1) {
                value = 1;
            }
            if (value < 0) {
                value = 0;
            }

            rgb = HSVtoRGB.convert(stateHue, saturation, value);

            rgb[0] = averageR.process(rgb[0]);
            rgb[1] = averageG.process(rgb[1]);
            rgb[2] = averageB.process(rgb[2]);

            setLEDs(rgb[0], rgb[1], rgb[2]);

        } else if (currentState == State.BREATHING_GREEN && currentState.isCycleColors == true) {
            if (startingTransTime <= currentState.transitionTime && !resetBreath) {
                startingTransTime += currentState.transitionTime / 50.0;
            } else if (resetBreath) {
                startingTransTime -= currentState.transitionTime / 50.0;
            }
            if (resetBreath && startingTransTime <= 0.0) {
                resetBreath = false;
            } else if (!resetBreath && startingTransTime >= currentState.transitionTime) {
                resetBreath = true;
            }

            float rgb[] = new float[3];
            MovingAverage averageR = new MovingAverage(10);
            MovingAverage averageG = new MovingAverage(10);
            MovingAverage averageB = new MovingAverage(10);

            double valueBasedOnTime = currentState.transitionTime - startingTransTime;

            rgb = HSVtoRGB.convert(State.BREATHING_GREEN.startingHue, 1.0f, valueBasedOnTime * 0.6);

            rgb[0] = averageR.process(rgb[0]);
            rgb[1] = averageG.process(rgb[1]);
            rgb[2] = averageB.process(rgb[2]);

            setLEDs(rgb[0], rgb[1], rgb[2]);

        } else if (!lit && (timestamp - lastOffTime) >= currentState.offTime && currentState.isCycleColors == false) {
            setLEDs(currentState.red, currentState.green, currentState.blue);
            lastOnTime = timestamp;
            lit = true;
        } else if (lit && !Double.isInfinite(currentState.onTime) && currentState.isCycleColors == false) {
            if ((timestamp - lastOnTime) >= currentState.onTime) {
                setLEDs(0.0, 0.0, 0.0);
                lastOffTime = timestamp;
                lit = false;
            }
        }
    }

    public void outputTelemetry() {
        SmartDashboard.putString("leds state", getState().name());
    }

    public boolean checkSystem() {
        return true;
    }
}