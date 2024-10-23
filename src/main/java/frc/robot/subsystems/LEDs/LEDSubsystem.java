// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
    public static CANifier canifier = new CANifier(Constants.LED_ID);

    private static LEDSubsystem instance = null;
    private State currentState = State.OFF;
    public double stateHue = State.RAINBOW.startingHue;
    public float saturation = 1.0f; // Ensures that the colors are on the outside of the color wheel
    public float value = 0.5f; // Hardcoded brightness for rainbow
    public double startingTransTime = 0.0;
    public boolean resetBreath = false;
    boolean lit = false;
    double lastOnTime = 0.0;
    double lastOffTime = 0.0;
    double transTime = 0.0;

    public static LEDSubsystem getInstance() {
        if (instance == null)
            instance = new LEDSubsystem();
        return instance;
    }

    public enum State {
        OFF(0.0, 0.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        SOLID_RED(255.0, 0.0, 0.0, Double.POSITIVE_INFINITY, 0, false),
        SOLID_ORANGE(255.0, 25.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        SOLID_YELLOW(255.0, 150.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        SOLID_GREEN(0.0, 255.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        SOLID_BLUE(0.0, 0.0, 255.0, Double.POSITIVE_INFINITY, 0.0, false),
        SOLID_PURPLE(255.0, 0.0, 255.0, Double.POSITIVE_INFINITY, 0.0, false),
        SOLID_PINK(255.0, 20.0, 30.0, Double.POSITIVE_INFINITY, 0.0, false),
        FLASHING_RED(255.0, 0.0, 0.0, 0.25, 0.25, false),
        FLASHING_ORANGE(255.0, 25.0, 0.0, 0.25, 0.25, false),
        FLASHING_YELLOW(255.0, 150.0, 0.0, 0.25, 0.25, false),
        FLASHING_GREEN(0.0, 255.0, 0.0, 0.25, 0.25, false),
        FLASHING_BLUE(0.0, 0.0, 255.0, 0.25, 0.25, false),
        FLASHING_PURPLE(255.0, 0.0, 255.0, 0.25, 0.25, false),
        FLASHING_PINK(255.0, 20.0, 30.0, 0.25, 0.25, false),
        RAINBOW(0, true),
        BREATHING_RED(0, 10.0, true),
        BREATHING_ORANGE(30, 10.0, true),
        BREATHING_YELLOW(60, 10.0, true),
        BREATHING_GREEN(120, 10.0, true),
        BREATHING_BLUE(240, 10.0, true),
        BREATHING_PURPLE(300, 10.0, true),
        BREATHING_PINK(360, 10.0, true);

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
        canifier.setLEDOutput(g, LEDChannel.LEDChannelA);
        canifier.setLEDOutput(r, LEDChannel.LEDChannelB);
        canifier.setLEDOutput(b, LEDChannel.LEDChannelC);
    }

    public void conformToState(State state) {
        setState(state);
    }

    @Override
    public void periodic() {
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

        } else if (isBreathing(currentState) && currentState.isCycleColors == true) {
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

            rgb = HSVtoRGB.convert(getState().startingHue, 1.0f, valueBasedOnTime * 0.6);

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

    private boolean isBreathing(State currentState) {
        if (currentState == State.BREATHING_RED || currentState == State.BREATHING_ORANGE
                || currentState == State.BREATHING_YELLOW || currentState == State.BREATHING_GREEN ||
                currentState == State.BREATHING_BLUE || currentState == State.BREATHING_PURPLE
                || currentState == State.BREATHING_PINK) {
            return true;
        } else {
            return false;
        }
    }
}