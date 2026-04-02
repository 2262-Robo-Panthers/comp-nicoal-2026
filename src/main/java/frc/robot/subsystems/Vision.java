// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
  private Drive m_drive;
  private ADIS16470_IMU m_gyro;

  private double m_heartbeat = 0.0;

  private Field2d m_fieldWidget = new Field2d();

  public Vision(ADIS16470_IMU gyro, Drive drive) {
    m_gyro = gyro;
    m_drive = drive;

    LimelightHelpers.setPipelineIndex("", 0);
    LimelightHelpers.setupPortForwardingUSB(0);

    // LimelightHelpers.setRewindEnabled("", true);
    LimelightHelpers.setRewindEnabled("", false);

    CameraServer.startAutomaticCapture();
  }

  @Override
  public void periodic() {
    SwerveDrivePoseEstimator poseEstimator = m_drive.getPoseEstimator();

    LimelightHelpers.SetRobotOrientation("", m_gyro.getAngle(IMUAxis.kZ), 0, 0, 0, 0, 0);
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

    if (Math.abs(m_gyro.getRate(IMUAxis.kZ)) < 720 && estimate.tagCount > 0) {
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
    }

    double heartbeat = LimelightHelpers.getHeartbeat("");
    SmartDashboard.putBoolean("Vision.LimelightOk", heartbeat != m_heartbeat);
    m_heartbeat = heartbeat;

    m_fieldWidget.setRobotPose(poseEstimator.getEstimatedPosition());
    SmartDashboard.putData("Vision.Field", m_fieldWidget);
  }
}

