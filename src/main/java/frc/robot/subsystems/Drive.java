// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.util.SwerveConstants.Drive.*;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.util.DriveFeedforwards;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.hal.*;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;

public class Drive extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_swerveFL = new MAXSwerveModule(20, 21, -Math.PI / 2);
  private final MAXSwerveModule m_swerveFR = new MAXSwerveModule(26, 31, 0.0);
  private final MAXSwerveModule m_swerveRL = new MAXSwerveModule(29, 30, Math.PI);
  private final MAXSwerveModule m_swerveRR = new MAXSwerveModule(22, 27, Math.PI / 2);

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Odometry class for tracking robot pose
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    kDriveKinematics,
    Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
    new SwerveModulePosition[] {
      m_swerveFL.getPosition(),
      m_swerveFR.getPosition(),
      m_swerveRL.getPosition(),
      m_swerveRR.getPosition()
    },
    new Pose2d(),
    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
  );

  public Drive() {
    // Usage reporting for MAXSwerve template
    HAL.report(
      FRCNetComm.tResourceType.kResourceType_RobotDrive,
      FRCNetComm.tInstances.kRobotDriveSwerve_MaxSwerve
    );
  }

  /*RobotConfig config = null;

    try {
      config = RobotConfig.fromGUISettings();
    }
    catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
      this::getPose,
      this::definePose,
      this::getModuleStates,
      this::driveRobotRelative,
      new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0)
      ), 
      config,
      () -> DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Blue,
      this
    );
  } */

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_poseEstimator.update(
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
        m_swerveFL.getPosition(),
        m_swerveFR.getPosition(),
        m_swerveRL.getPosition(),
        m_swerveRR.getPosition()
      }
    );
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void definePose(Pose2d pose) {
    m_poseEstimator.resetPosition(
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
        m_swerveFL.getPosition(),
        m_swerveFR.getPosition(),
        m_swerveRL.getPosition(),
        m_swerveRR.getPosition()
      },
      pose
    );
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return m_poseEstimator;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * kMaxAngularSpeed;

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xSpeedDelivered, ySpeedDelivered, rotDelivered,
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ))
    );

    driveRobotRelative(speeds, null);
  }

  public void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    ChassisSpeeds discretized = ChassisSpeeds.discretize(speeds, 0.02);

    SwerveModuleState[] states = kDriveKinematics.toSwerveModuleStates(discretized);
    setModuleStates(states);
  }

  public Command cmd_manualDrive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot) {
    return run(() -> drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble()));
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_swerveFL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_swerveFR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_swerveRL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_swerveRR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, kMaxSpeedMetersPerSecond);
    m_swerveFL.setDesiredState(desiredStates[0]);
    m_swerveFR.setDesiredState(desiredStates[1]);
    m_swerveRL.setDesiredState(desiredStates[2]);
    m_swerveRR.setDesiredState(desiredStates[3]);
  }

  public ChassisSpeeds getModuleStates() {
    return kDriveKinematics.toChassisSpeeds(
      m_swerveFL.getState(),
      m_swerveFR.getState(),
      m_swerveRL.getState(),
      m_swerveRR.getState()
    );
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_swerveFL.resetDriveEncoder();
    m_swerveRL.resetDriveEncoder();
    m_swerveFR.resetDriveEncoder();
    m_swerveRR.resetDriveEncoder();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }
}
