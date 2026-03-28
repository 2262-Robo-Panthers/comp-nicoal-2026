// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.*;

public class Shooter extends SubsystemBase {
  // private AngularVelocity m_flywheelSpeed, m_feedSpeed, m_shootableThreshold;

  private SparkFlex m_ctrlA = new SparkFlex(39, SparkFlex.MotorType.kBrushless);
  private SparkFlex m_ctrlB = new SparkFlex(40, SparkFlex.MotorType.kBrushless);

  private SparkClosedLoopController m_flywheel = m_ctrlA.getClosedLoopController();

  private SparkMax m_ctrlC = new SparkMax(8, SparkMax.MotorType.kBrushless);

  private SparkClosedLoopController m_feed = m_ctrlC.getClosedLoopController();

  private Servo m_servo = new Servo(0);
  private double m_hoodPosition = 0.265;

  public Shooter() {
  // public Shooter(AngularVelocity flywheelSpeed, AngularVelocity feedSpeed, AngularVelocity shootableThreshold) {
    // m_flywheelSpeed = flywheelSpeed;
    // m_feedSpeed = feedSpeed;
    // m_shootableThreshold = shootableThreshold;

    configureFlywheel();
    configureFeed();
    configureHood();
  }

  private void configureFlywheel() {
    SparkFlexConfig configA = new SparkFlexConfig();

    configA
      .inverted(true)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(40);
    configA.encoder
      .positionConversionFactor(1.0)
      .velocityConversionFactor(1.0);
    configA.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(-1.0, 1.0)
      .pid(0.0, 0.0, 0.0);

    m_ctrlA.configure(
      configA,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    SparkFlexConfig configB = new SparkFlexConfig();

    configB
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(40)
      .follow(m_ctrlA, true);

    m_ctrlB.configure(
      configB,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
  }

  private void configureFeed() {
    SparkMaxConfig configC = new SparkMaxConfig();

    configC
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(20);
    configC.encoder
      .positionConversionFactor(0.1)
      .velocityConversionFactor(0.1);
    configC.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(-1.0, 1.0)
      .pid(0.0, 0.0, 0.0);

    m_ctrlC.configure(
      configC,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
  }

  public void configureHood() {
    m_servo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
  }

  public Command cmd_manual(DoubleSupplier flywheel, DoubleSupplier feed, DoubleSupplier srv) {
    return run(() -> {
      this.flywheel(flywheel.getAsDouble());
      this.feed(feed.getAsDouble());
      double delta = (1.0/50) * 0.5 * srv.getAsDouble();
      if (m_hoodPosition >= 0.5 && delta > 0.0 ||
        m_hoodPosition <= 0.0 && delta < 0.0)
        delta = 0;
      m_hoodPosition += delta;
      m_servo.set(m_hoodPosition);
    })
      .withName("Manual Control");
  }

  public void flywheel(double speed) {
    // m_flywheel.setSetpoint(speed * m_flywheelSpeed.in(Revolutions.per(Minute)), ControlType.kVelocity);
    m_ctrlA.set(speed*0.67);
  }

  public void feed(double speed) {
    // m_feed.setSetpoint(speed * m_feedSpeed.in(Revolutions.per(Minute)), ControlType.kVelocity);
    m_ctrlC.set(speed);
  }

  public boolean isAtSpeed() {
    // return Math.abs(m_ctrlA.getEncoder().getVelocity() - m_flywheel.getSetpoint())
    //   <= m_shootableThreshold.in(Revolutions.per(Minute));
    return true;
  }

  public Command cmd_waitSpinUp() {
    return new WaitUntilCommand(this::isAtSpeed)
      .beforeStarting(runOnce(() -> flywheel(1.0)))
      .withTimeout(3.0)
      .withName("Spinning Up");
  }

  public Command cmd_manualShoot(double flywheel, DoubleSupplier feed) {
    return run(() -> feed(feed.getAsDouble()))
      .beforeStarting(cmd_waitSpinUp())
      .withName("Manual Shooting");
  }

  public Command cmd_stop() {
    return runOnce(() -> {
      flywheel(0.0);
      feed(0.0);
    });
  }

  @Override
  public void periodic() {
    Command foo = getCurrentCommand();
    SmartDashboard.putString("Shooter.Command", foo == null ? "none" : foo.getName());
    SmartDashboard.putNumber("Shooter.Speed", m_ctrlA.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter.Position", m_hoodPosition);
  }
}
