// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import modulelib.SwerveModule;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModuleLocations;
import frc.robot.Constants.DriveConstants.SwerveModules;
import frc.robot.Constants.PathPlannerConstants.RotationPID;
import frc.robot.Constants.PathPlannerConstants.TranslationPID;

public class SwerveSubsystem extends SubsystemBase {
  private AHRS gyro;

  private SwerveModule frontLeft = new SwerveModule(SwerveModules.frontLeft,
      Constants.DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND, Constants.DriveConstants.MAX_VOLTAGE);
  private SwerveModule frontRight = new SwerveModule(SwerveModules.frontRight,
      Constants.DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND, Constants.DriveConstants.MAX_VOLTAGE);
  private SwerveModule backLeft = new SwerveModule(SwerveModules.backLeft,
      Constants.DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND, Constants.DriveConstants.MAX_VOLTAGE);
  private SwerveModule backRight = new SwerveModule(SwerveModules.backRight,
      Constants.DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND, Constants.DriveConstants.MAX_VOLTAGE);

  private SwerveModule[] modules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      ModuleLocations.frontLeft,
      ModuleLocations.frontRight,
      ModuleLocations.backLeft,
      ModuleLocations.backRight);

  private SwerveDriveOdometry odometry;

  private Field2d field = new Field2d();

  private SlewRateLimiter slewRateX = new SlewRateLimiter(DriveConstants.slewRate);
  private SlewRateLimiter slewRateY = new SlewRateLimiter(DriveConstants.slewRate);

  private BooleanEntry fieldOrientedEntry;

  /** Creates a new Drivebase. */
  public SwerveSubsystem(AHRS gyro) {
    var inst = NetworkTableInstance.getDefault();
    var table = inst.getTable("SmartDashboard");
    this.fieldOrientedEntry = table.getBooleanTopic("Field Oriented").getEntry(true);

    this.gyro = gyro;
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry
        this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(TranslationPID.p, TranslationPID.i, TranslationPID.d), // Translation PID constants
            new PIDConstants(RotationPID.p, RotationPID.i, RotationPID.d), // Rotation PID constants
            Constants.DriveConstants.AUTO_MAX_SPEED_METERS_PER_SECOND, // Max module speed, in m/s
            ModuleLocations.robotRaduius, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig()), // Default path replanning config. See the API for the options here

        // Boolean supplier that controls when the path will be mirrored for the red
        // alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        () -> {
          // var alliance = DriverStation.getAlliance();
          // if (alliance.isPresent()) {
          // return alliance.get() == DriverStation.Alliance.Red;
          // }
          return false;
        },
        this); // Reference to this subsystem to set requirements

    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    SmartDashboard.putData("Field", field);
  }

  public double getFieldAngle() {
    return -gyro.getYaw();
  }

  public void fieldOrientedDrive(double speedX, double speedY, double rot) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, rot,
        Rotation2d.fromDegrees(getFieldAngle()));
    this.drive(speeds);
  }

  public void robotOrientedDrive(double speedX, double speedY, double rot) {
    ChassisSpeeds speeds = new ChassisSpeeds(speedX, speedY, rot);
    this.drive(speeds);
  }

  public void defaultDrive(double speedX, double speedY, double rot) {
    defaultDrive(speedX, speedY, rot, true);
  }

  public void defaultDrive(double speedX, double speedY, double rot, boolean slew) {
    if (slew) {
      speedX = slewRateX.calculate(speedX);
      speedY = slewRateY.calculate(speedY);
    }

    if (this.fieldOrientedEntry.get(true)) {
      fieldOrientedDrive(speedX, speedY, rot);
    } else {
      robotOrientedDrive(speedX, speedY, rot);
    }
  }

  private void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds,
        new Translation2d(Units.inchesToMeters(4), 0));

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
        Constants.DriveConstants.TELEOP_MAX_SPEED_METERS_PER_SECOND);

    this.frontLeft.drive(moduleStates[0]);
    this.frontRight.drive(moduleStates[1]);
    this.backLeft.drive(moduleStates[2]);
    this.backRight.drive(moduleStates[3]);

    SmartDashboard.putNumber("FL Target Angle", moduleStates[0].angle.getDegrees());
  }

  public double getMaxVelocity() {
    return Constants.DriveConstants.TELEOP_MAX_SPEED_METERS_PER_SECOND;
  }

  public double getMaxAngleVelocity() {
    return Constants.DriveConstants.teleOpNormalAngularSpeed;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose2d) {
    odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose2d);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public void setSpeed(double vx, double vy, double omegaDegreesPerSecond, boolean fieldOriented) {
    setSpeed(new ChassisSpeeds(vx, vy, Units.degreesToRadians(omegaDegreesPerSecond)), fieldOriented);
  }

  public void setSpeed(ChassisSpeeds chassisSpeeds, boolean fieldOriented) {
    if (fieldOriented) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
          chassisSpeeds.omegaRadiansPerSecond, getPose().getRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
          chassisSpeeds.omegaRadiansPerSecond);
    }

    // This will take the speeds that we want our robot to move and turn at, and
    // calculate the required direction and speed for each swerve module on the
    // robot.
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    // Set the swerve modules to their required states.
    setModuleStates(moduleStates);
  }

  @Override
  public void periodic() {
    var positions = getPositions();

    odometry.update(gyro.getRotation2d(), positions);

    field.setRobotPose(getPose());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
        Constants.DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
  }

  public void stopModules() {
    setSpeed(0, 0, 0, true);
  }
}