package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.SwerveConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.*;

public class MAXSwerveModule {
  private double m_chassisAngularOffset;

  private SparkMax m_drive, m_steer;
  private RelativeEncoder m_driveEncoder;
  private AbsoluteEncoder m_steerEncoder;
  private SparkClosedLoopController m_driveCtrl, m_steerCtrl;

  public MAXSwerveModule(int driveId, int steerId, double chassisAngularOffset) {
    m_chassisAngularOffset = chassisAngularOffset;

    m_drive = new SparkMax(driveId, SparkMax.MotorType.kBrushless);
    m_steer = new SparkMax(steerId, SparkMax.MotorType.kBrushless);

    m_driveEncoder = m_drive.getEncoder();
    m_steerEncoder = m_steer.getAbsoluteEncoder();

    m_driveCtrl = m_drive.getClosedLoopController();
    m_steerCtrl = m_steer.getClosedLoopController();

    SparkMaxConfig driveConfig = new SparkMaxConfig();

    driveConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(35); // TODO why 50 if the breaker is 40?
    driveConfig.encoder
      .positionConversionFactor(SwerveConstants.Module.kDrivingFactor)
      .velocityConversionFactor(SwerveConstants.Module.kDrivingFactor / 60.0);
    driveConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(-1.0, 1.0)
      .pid(0.04, 0.0, 0.0)
      .feedForward.kV(SwerveConstants.Module.kDrivingVelocityFeedForward);

    m_drive.configure(
      driveConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    SparkMaxConfig steerConfig = new SparkMaxConfig();

    steerConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(20);
    steerConfig.absoluteEncoder
      .inverted(true)
      .positionConversionFactor(SwerveConstants.Module.kTurningFactor)
      .velocityConversionFactor(SwerveConstants.Module.kTurningFactor / 60.0)
      .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);
    steerConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .outputRange(-1.0, 1.0)
      .pid(0.5, 0.0, 0.0)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(0.0, SwerveConstants.Module.kTurningFactor);

    m_steer.configure(
      steerConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_driveEncoder.setPosition(0.0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      m_driveEncoder.getVelocity(),
      new Rotation2d(m_steerEncoder.getPosition() - m_chassisAngularOffset)
    );
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      m_driveEncoder.getPosition(),
      new Rotation2d(m_steerEncoder.getPosition() - m_chassisAngularOffset)
    );
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = desiredState;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
    correctedDesiredState.optimize(new Rotation2d(m_steerEncoder.getPosition()));

    m_driveCtrl.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_steerCtrl.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
  }

  public void resetDriveEncoder() {
    m_driveEncoder.setPosition(0.0);
  }
}
