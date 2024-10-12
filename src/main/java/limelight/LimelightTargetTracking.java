// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class LimelightTargetTracking {

    private static final String LIMELIGHT_TABLE_NAME = "limelight";
    private static final String LIMELIGHT_TABLE_ENTRY_X = "tx";
    private static final String LIMELIGHT_TABLE_ENTRY_Y = "ty";

    // tv = 0 if no valid targets identified, tv = 1 for valid target
    private static final String LIMELIGHT_TABLE_ENTRY_VALID = "tv";

    private final LimelightConfiguration configuration;
    private final NetworkTable table;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    public final NetworkTableEntry tv;

    public LimelightTargetTracking(LimelightConfiguration configuration) {
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

    // TO-DO fix these
    public double computePivotAngle(double distance) {
        return 147.1408 - 3.172868 * distance + 0.03825801 * distance * distance
                - 0.0002114852 * distance * distance * distance + 0.0000004285076 * distance * distance * distance * distance;
    } // Current values- 68 at 41, 49 at 78, 42 at 140, 33.5 at 168, 40 at 137
    // New values- 68 at 41, 49 at 74, 42.6 at 108, 37 at 140.5, 32 at 179,

    public double computeShooterRightSpeed(double distance) {
        return 1362.989 + 38.67915 * distance - 0.3083168 * distance * distance
                + 0.0010076 * distance * distance * distance;
    } // 2500 at 41, 3000 at 80, 3500 at 140, 4000 at 171, 4000 at 137

    public double computeShooterLeftSpeed(double distance) {
        return -33.90943 + 76.20024 * distance - 0.7453442 * distance * distance
                + 0.002359556 * distance * distance * distance;
    } // 2000 at 41, 2500 at 80, 2500 at 140, 3000 at 171, 3000 at 137

    public double getAngleToSpeaker() {
        return getTx();
      }
}