// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;
import frc.robot.constants.PathPlannerConstants;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.constants.SwerveDriveConstants.SwerveModuleConfigurations;

/**
 * Basic simulation of a swerve subsystem with the methods needed by PathPlanner
 */
public class PathSwerveSubsystem extends SubsystemBase {
  private SwerveModule[] modules;
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;

  private final AHRS gyro = new AHRS(IDs.AHRS_PORT_ID);

  private Field2d field = new Field2d();

  public PathSwerveSubsystem() {
    modules = new SwerveModule[] {
        new SwerveModule(SwerveModuleConfigurations.values()[0]),
        new SwerveModule(SwerveModuleConfigurations.values()[1]),
        new SwerveModule(SwerveModuleConfigurations.values()[2]),
        new SwerveModule(SwerveModuleConfigurations.values()[3])
    };
    kinematics = new SwerveDriveKinematics(
        PathPlannerConstants.Swerve.flModuleOffset,
        PathPlannerConstants.Swerve.frModuleOffset,
        PathPlannerConstants.Swerve.blModuleOffset,
        PathPlannerConstants.Swerve.brModuleOffset);
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());

    // Configure AutoBuilder
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getSpeeds,
        this::driveRobotRelative,
        PathPlannerConstants.Swerve.pathFollowerConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    odometry.update(getRotation2d(), getPositions());

    field.setRobotPose(getPose());
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public double getHeading() {
    // Because the NavX gives headings from -180 to 180 degrees, we need to convert
    // it to a range of 0 to 360 degrees.
    // negative because we need CCW = positive
    double angle = -gyro.getYaw();
    boolean sign = angle < 0;
    angle = Math.abs(angle);
    angle = angle % 360;
    if (sign)
      angle = 360 - angle;
    return angle;
    // return Math.IEEEremainder(-gyro.getAngle(), 360);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void setBrakeMode(boolean brake) {
    for (SwerveModule module : modules) {
      module.setBrakeMode(brake);
    }
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates,
        SwerveDriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(targetStates[i]);
    }
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
}