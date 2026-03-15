// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.PneumaticsModuleType.REVPH;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.*;

// TODO add compressor data and solenoid and roller status to Elastic

public class Intake extends SubsystemBase {
  private double m_speed;

  private SparkMax m_rollers = new SparkMax(61, SparkMax.MotorType.kBrushed);

  private Compressor m_compressor = new Compressor(REVPH);
  private DoubleSolenoid m_slndA = new DoubleSolenoid(REVPH, 0, 1);
  private DoubleSolenoid m_slndB = new DoubleSolenoid(REVPH, 2, 3);

  public Intake(double speed) {
    m_speed = speed;

    SparkMaxConfig config = new SparkMaxConfig();

    config
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(20);

    m_rollers.configure(
      config,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
  }

  public Command cmd_setExtension(boolean extended) {
    DoubleSolenoid.Value value = extended ? kForward : kReverse;

    return runOnce(() -> {
      m_slndA.set(value);
      m_slndB.set(value);
    });
  }

  public Command cmd_setRollers(boolean on) {
    return runOnce(() -> {
      m_rollers.set(on ? m_speed : 0.0);
    });
  }

  public Command cmd_toggleRollers() {
    return runOnce(() -> {
      if (m_rollers.get() == 0.0)
        m_rollers.set(m_speed);
      else
        m_rollers.set(0.0);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
