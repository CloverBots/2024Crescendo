// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class VisionTargetTracker {

    private static final String LIMELIGHT_TABLE_NAME = "limelight";
    private static final String LIMELIGHT_TABLE_ENTRY_X = "tx";
    private static final String LIMELIGHT_TABLE_ENTRY_Y = "ty";

    // tv = 0 if no valid targets identified, tv = 1 for valid target
    private static final String LIMELIGHT_TABLE_ENTRY_VALID = "tv";

    private final VisionConfiguration configuration;
    private final NetworkTable table;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry tv;

    public VisionTargetTracker(VisionConfiguration configuration) {
        this.configuration = configuration;

        table = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_NAME);
        tx = table.getEntry(LIMELIGHT_TABLE_ENTRY_X);
        ty = table.getEntry(LIMELIGHT_TABLE_ENTRY_Y);
        tv = table.getEntry(LIMELIGHT_TABLE_ENTRY_VALID);
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
        return 78.658 - 0.3889 * distance + 0.0007654 * distance * distance;
    }

    public double computeShooterRightSpeed(double distance) {
        return 2698.927 - 10.32823 * distance + 0.145699 * distance * distance
                - 0.0002956 * distance * distance * distance;
    }

    public double computeShooterLeftSpeed(double distance) {
        return 3370.554 - 55.07335 * distance + 0.5782 * distance * distance
                - 0.0014495 * distance * distance * distance;
    }
}
