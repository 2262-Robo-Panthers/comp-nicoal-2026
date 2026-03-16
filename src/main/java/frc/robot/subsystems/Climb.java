// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.*;

// TODO add to Elastic: setpoint, actual position, isAtPosition

public class Climb extends SubsystemBase {
  private Distance m_travelLength;
  private static final Distance kCircumference = Inches.of(Math.PI * 1.8);

  private double m_setpoint = 0.0;

  private SparkMax m_ctrlA = new SparkMax(17, SparkMax.MotorType.kBrushless);
  private SparkMax m_ctrlB = new SparkMax(18, SparkMax.MotorType.kBrushless);

  private SparkClosedLoopController m_elevator = m_ctrlA.getClosedLoopController();

  public Climb(Distance travelLength) {
    m_travelLength = travelLength;

    SparkMaxConfig configA = new SparkMaxConfig();

    configA
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40);
    configA.encoder
      .positionConversionFactor(0.0625 / (m_travelLength.div(kCircumference)).magnitude());
    configA.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(-1.0, 1.0)
      .pid(0.0, 0.0, 0.0);
    configA.closedLoop.maxMotion
      .cruiseVelocity(6.0 / kCircumference.magnitude()) // TODO tune these
      .maxAcceleration(12.0 / kCircumference.magnitude())
      .allowedProfileError(0.5);

    m_ctrlA.configure(
      configA,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    SparkMaxConfig configB = new SparkMaxConfig();

    configB
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40)
      .follow(m_ctrlA, false);

    m_ctrlB.configure(
      configB,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_ctrlA.getEncoder().setPosition(0.0);
  }

  public Command cmd_moveSetpoint(DoubleSupplier delta) {
    return run(() -> moveSetpoint(delta.getAsDouble()));
  }

  public void setSetpoint(double setpoint) {
    m_setpoint = Math.max(0.0, Math.min(setpoint, 1.0));
    m_elevator.setSetpoint(setpoint, ControlType.kMAXMotionPositionControl);
  }

  public void moveSetpoint(double delta) {
    setSetpoint(m_setpoint + delta);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
