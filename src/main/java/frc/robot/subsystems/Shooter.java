// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.*;

// TODO add to Elastic: current RPMs, isAtSpeed, setpoints

public class Shooter extends SubsystemBase {
  private double m_flywheelRpm, m_feedRpm, m_shootableThreshold;

  private SparkFlex m_ctrlA = new SparkFlex(39, SparkFlex.MotorType.kBrushless);
  private SparkFlex m_ctrlB = new SparkFlex(40, SparkFlex.MotorType.kBrushless);

  private SparkClosedLoopController m_flywheel = m_ctrlA.getClosedLoopController();

  private SparkMax m_ctrlC = new SparkMax(8, SparkMax.MotorType.kBrushless);

  private SparkClosedLoopController m_feed = m_ctrlC.getClosedLoopController();

  public Shooter(double flywheelRpm, double feedRpm, double shootableThreshold) {
    m_flywheelRpm = flywheelRpm;
    m_feedRpm = feedRpm;
    m_shootableThreshold = shootableThreshold;

    configureFlywheel();
    configureFeed();
  }

  private void configureFlywheel() {
    SparkFlexConfig configA = new SparkFlexConfig();

    configA
      .idleMode(IdleMode.kCoast) // TODO need to change this or can it active brake when necessary?
      .smartCurrentLimit(20);
    // configA.encoder
    //   .positionConversionFactor(1.0)
    //   .velocityConversionFactor(1.0); // TODO is this necessary?
    configA.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(-1.0, 1.0)
      .pid(0.0, 0.0, 0.0); // TODO is PID necessary? tune if so
    configA.closedLoop.maxMotion
      .cruiseVelocity(m_flywheelRpm) // TODO tune these
      .maxAcceleration(500.0)
      .allowedProfileError(1.0);

    m_ctrlA.configure(
      configA,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    SparkFlexConfig configB = new SparkFlexConfig();

    configB
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(20)
      .follow(m_ctrlA);

    m_ctrlB.configure(
      configB,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
  }

  private void configureFeed() {
    SparkMaxConfig configC = new SparkMaxConfig();

    configC
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(20);
    configC.encoder
      .positionConversionFactor(0.1)
      .velocityConversionFactor(0.1);
    configC.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(-1.0, 1.0)
      .pid(0.5, 0.0, 0.0);
    configC.closedLoop.maxMotion
      .cruiseVelocity(m_feedRpm) // TODO tune these
      .maxAcceleration(5.0)
      .allowedProfileError(0.1);

    m_ctrlC.configure(
      configC,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
  }

  public Command cmd_manual(DoubleSupplier flywheel, DoubleSupplier feed) {
    return run(() -> {
      flywheel(flywheel.getAsDouble());
      feed(feed.getAsDouble());
    })
      .withName("Manual Control");
  }

  public Command cmd_flywheel(double speed) {
    return runOnce(() -> flywheel(speed));
  }

  public void flywheel(double speed) {
    m_flywheel.setSetpoint(speed * m_flywheelRpm, ControlType.kMAXMotionVelocityControl);
  }

  public Command cmd_feed(double speed) {
    return runOnce(() -> feed(speed));
  }

  public void feed(double speed) {
    m_feed.setSetpoint(speed * m_feedRpm, ControlType.kMAXMotionVelocityControl);
  }

  public boolean isAtSpeed() {
    return Math.abs(m_ctrlA.getEncoder().getVelocity() - m_flywheel.getSetpoint())
      <= m_shootableThreshold;
  }

  public Command cmd_waitSpinUp() {
    return new WaitUntilCommand(this::isAtSpeed)
      .beforeStarting(cmd_flywheel(1.0))
      .withTimeout(3.0)
      .withName("Spinning Up");
  }

  public Command cmd_manualShoot(double flywheel, DoubleSupplier feed) {
    return run(() -> feed(feed.getAsDouble()))
      .beforeStarting(cmd_waitSpinUp())
      .withName("Manual Shooting");
  }

  public Command cmd_autoShoot(double flywheel) {
    return startEnd(() -> feed(1.0), () -> feed(0.0))
      .beforeStarting(cmd_waitSpinUp())
      .withName("Auto Shooting");
  }

  public Command cmd_stop() {
    return runOnce(() -> {
      m_flywheel.setSetpoint(0.0, ControlType.kMAXMotionVelocityControl);
      m_feed.setSetpoint(0.0, ControlType.kMAXMotionVelocityControl);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
