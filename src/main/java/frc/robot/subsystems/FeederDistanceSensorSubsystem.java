package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederDistanceSensorSubsystem extends SubsystemBase {
    
    TimeOfFlight distanceSensor = new TimeOfFlight(0);

    public static final double NOTE_LOADED_DISTANCE = 160; //185

    public FeederDistanceSensorSubsystem() {
        distanceSensor.setRangingMode(RangingMode.Short, 24);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ToF", distanceSensor.getRange());
        SmartDashboard.putBoolean("Is Note Loaded", isNoteLoaded());
    }

    public boolean isNoteLoaded() {
        if (distanceSensor.getRange() < NOTE_LOADED_DISTANCE ) {
            return true;
        } else {
            return false;
        }
    }

   
}