package frc.robot;

/* Camera is configured with 1280x800 120fps, 840 Exposure, 10 Black Level Offset, 6 Sensor Gain, 1200 Red Balance
 * 1975 Blue Balance, Normal Stream Rate, and 320x240
 */

public class VisionConfiguration {
    private final double targetHeight;
    private final double cameraHeight;
    private final double cameraPitch;

    /**
     * Constructs a new {@link VisionConfiguration} instance.
     * @param targetHeight The height of the target, usually in inches.
     * @param cameraHeight The height of the camera off the ground, usually in inches.
     * @param cameraPitch The pitch of the camera in degrees.
     */
    public VisionConfiguration(double targetHeight, double cameraHeight, double cameraPitch) {
        this.targetHeight = targetHeight;
        this.cameraHeight = cameraHeight;
        this.cameraPitch = cameraPitch;
    }

    /**
     * Gets the height of the target, usually in inches.
     */
    public double getTargetHeight() {
        return targetHeight;
    }

    /**
     * Gets the height of the camera, usually in inches.
     */
    public double getCameraHeight() {
        return cameraHeight;
    }

    /**
     * Gets the pitch of the camera in degrees.
     */
    public double getCameraPitch() {
        return cameraPitch;
    }
}
