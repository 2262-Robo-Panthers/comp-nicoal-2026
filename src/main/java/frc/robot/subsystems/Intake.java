// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.PneumaticsModuleType.REVPH;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.*;

public class Intake extends SubsystemBase {
  private double m_speed;

  private SparkMax m_rollers = new SparkMax(61, SparkMax.MotorType.kBrushed);

  private Compressor m_compressor = new Compressor(11, REVPH);
  private DoubleSolenoid m_solenoid = new DoubleSolenoid(11, REVPH, 7, 6);

  public Intake(double speed) {
    m_speed = speed;

    m_solenoid.set(kReverse);

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

    return runOnce(() -> m_solenoid.set(value));
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
    SmartDashboard.putBoolean("Intake.Extended", m_solenoid.get() == kForward);
    SmartDashboard.putBoolean("Intake.Spinning", m_rollers.get() > 0.0);
    SmartDashboard.putBoolean("Intake.TankFull", !m_compressor.getPressureSwitchValue());
  }
}
