// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class VisionTargetTracker {

    private static final String LIMELIGHT_TABLE_NAME = "limelight";
    private static final String LIMELIGHT_TABLE_ENTRY_X = "tx";
    private static final String LIMELIGHT_TABLE_ENTRY_Y = "ty";
    private static final String LIMELIGHT_TABLE_ENTRY_LIGHTS = "ledMode";

    // tv = 0 if no valid targets identified, tv = 1 for valid target
    private static final String LIMELIGHT_TABLE_ENTRY_VALID = "tv";

    private final VisionConfiguration configuration;
    private final NetworkTable table;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry tv;
    private final NetworkTableEntry lights;

    public VisionTargetTracker(VisionConfiguration configuration) {
        this.configuration = configuration;

        table = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_NAME);
        tx = table.getEntry(LIMELIGHT_TABLE_ENTRY_X);
        ty = table.getEntry(LIMELIGHT_TABLE_ENTRY_Y);
        tv = table.getEntry(LIMELIGHT_TABLE_ENTRY_VALID);
        lights = table.getEntry(LIMELIGHT_TABLE_ENTRY_LIGHTS);
    }

    public double getTx() {
        return tx.getDouble(0.0);
    }

    /**
     * Gets the center Y offset of the vision target in degrees
     * ranging from -20.5 to 20.5.
     */
    public double getTy() {
        return ty.getDouble(0.0);
    }

    /**
     * Get whether there is a valid target visible
     */
    public boolean isValid() {
        if (tv.getNumber(0).intValue() == 0) {
            return false;
        }

        return true;
    }

    /**
     * Computes the distance to the target, usually in inches.
     * 
     * @param config The configuration describing camera and field measurements.
     */
    public double computeTargetDistance() {
        if (isValid()) {
            var targetHeight = configuration.getTargetHeight();
            var cameraHeight = configuration.getCameraHeight();
            var cameraPitch = configuration.getCameraPitch();

            // convert degrees to radians because Math.tan uses radians
            return (targetHeight - cameraHeight) / Math.tan(Math.toRadians(cameraPitch + getTy()));
        } else {
            return -1;
        }
    }

    public double computePivotAngle(double distance) {
        return 108.2513 - 1.482468 * distance + 0.01313622 * distance * distance - 0.00005496728 * distance * distance * distance + 0.00000008362174 * distance * distance * distance * distance; // 95.9703 - 0.834142 * distance + 0.004755 * distance * distance
                                                               //- 0.00001107 * distance * distance * distance
    } // New values- 68 at 41*, 49 at 78*, 42 at 140*, 33.5 at 168*, 40 at 137, 52 at 70, 42 at 119, 37 at 160

    public double computeShooterRightSpeed(double distance) {
        return 1362.989 + 38.67915 * distance - 0.3083168 * distance * distance
                + 0.0010076 * distance * distance * distance + 500;
    } // 2500 at 41, 3000 at 80, 3500 at 140, 4000 at 171, 4000 at 137

    public double computeShooterLeftSpeed(double distance) {
        return -33.90943 + 76.20024 * distance - 0.7453442 * distance * distance
                + 0.002359556 * distance * distance * distance + 500;
    } // 2000 at 41, 2500 at 80, 2500 at 140, 3000 at 171, 3000 at 137
}
