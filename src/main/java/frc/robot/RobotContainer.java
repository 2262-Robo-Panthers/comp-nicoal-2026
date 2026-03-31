// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import frc.robot.auto.Auto;
import frc.robot.subsystems.*;

public class RobotContainer {
  private Drive m_drive = new Drive();

  private Intake m_intake = new Intake(
    /* roller strength */ 1.0
  );

  private Shooter m_shooter = new Shooter();

  /*** private Climb m_climb = new Climb(
    Inches.of(12.0) // TODO measure this
  ); */

  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

  private Auto m_auto = new Auto(
    Commands.none()
      .withName("Do Nothing"),
    // TODO add autonomous commands
    m_drive.cmd_manualDrive(() -> 1.0, () -> 0.0, () -> 0.0)
      .raceWith(Commands.waitSeconds(3.0))
      .withName("Drive Forward 3s"),
      // Add auto
    Commands.sequence(
      m_shooter.cmd_manualShoot(1.0, () -> 0.0)
        .raceWith(Commands.waitSeconds(3.0)),
      m_shooter.cmd_manualShoot(1.0, () -> 1.0)
        .raceWith(Commands.waitSeconds(6.0)),
      m_shooter.cmd_stop()
    ).withName("Shoot")
  );

  private CommandXboxController m_driver = new CommandXboxController(0);
  private CommandXboxController m_operator = new CommandXboxController(1);

  private static final double kControllerDeadband = 0.07;

  public RobotContainer() {
    configureBindings();
    
    m_pdh.setSwitchableChannel(true);

    SmartDashboard.putData("Choose Auto", m_auto.getChooser());
  }

  private void configureBindings() {
    /*** Drive */
    {
      m_drive.setDefaultCommand(
        m_drive.cmd_manualDrive(
          () -> MathUtil.applyDeadband(-m_driver.getRightY(), kControllerDeadband),
          () -> MathUtil.applyDeadband(-m_driver.getRightX(), kControllerDeadband),
          () -> MathUtil.applyDeadband(-m_driver.getLeftX(), kControllerDeadband))
      );

      // Press Back to define current heading as forward

      m_driver.back()
        .onTrue(
          new InstantCommand(() -> m_drive.zeroHeading(), m_drive)
        );

      // Hold Start for X-formation

      m_driver.start()
        .whileTrue(
          new StartEndCommand(() -> m_drive.setX(), () -> {}, m_drive)
        );
    }

    /*** Intake */
    {
      // Press Back to toggle rollers

      m_operator.back()
        .onTrue(
          m_intake.cmd_toggleRollers()
        );

      // Press A to retract; with right bumper to also disable rollers

      m_operator.a()
        .onTrue(
          m_intake.cmd_setExtension(false)
        );

      m_operator.a().and(m_operator.rightBumper())
        .onTrue(
          m_intake.cmd_setRollers(false)
        );

      // Press B to extend; with right bumper to also enable rollers

      m_operator.b()
        .onTrue(
          m_intake.cmd_setExtension(true)
        );

      m_operator.b().and(m_operator.rightBumper())
        .onTrue(
          m_intake.cmd_setRollers(true)
        );
    }

    /*** Shooter */
    {
      m_shooter.setDefaultCommand(
        m_shooter.cmd_manual(
          () -> MathUtil.applyDeadband(-m_operator.getLeftY() * 0.85, kControllerDeadband),
          () -> MathUtil.applyDeadband(-m_operator.getRightY(), kControllerDeadband),
          () -> (
            m_operator.getHID().getPOV() == 180 ?
            m_operator.getRightTriggerAxis() - m_operator.getLeftTriggerAxis() :
            0.0
          ))
      );
      // Hood up/down with D-pad

      // Hold X to shoot (controlling the feed); press with right bumper to lock

      //D pad left 100% power
      m_operator.povLeft().and(m_operator.rightBumper().negate())
        .whileTrue(
          m_shooter.cmd_manualShoot(1.0, () -> -m_operator.getRightY()) // TODO change speed depending on distance
        );

      m_operator.povLeft().and(m_operator.rightBumper())
        .onTrue(
          m_shooter.cmd_manualShoot(1.0, () -> -m_operator.getRightY()) // TODO change speed depending on distance
        );

      //D pad right 70% power
      m_operator.povRight().and(m_operator.rightBumper().negate())
        .whileTrue(
          m_shooter.cmd_manualShoot(0.7, () -> -m_operator.getRightY())
        );

      m_operator.povRight().and(m_operator.rightBumper())
        .onTrue(
          m_shooter.cmd_manualShoot(0.7, () -> -m_operator.getRightY())
        );

      // Press Start to stop the whole shooter

      m_operator.start()
        .onTrue(
          m_shooter.cmd_stop()
        );
    }

    /*** Climb */
    /*** {
      m_climb.setDefaultCommand(
        m_climb.cmd_moveSetpoint(() ->
          m_operator.getHID().getPOV() == 90 ?
          MathUtil.applyDeadband(m_operator.getRightTriggerAxis() - m_operator.getLeftTriggerAxis(), kControllerDeadband) :
          0.0
        ));
    }*/
  }

  public Command getAutonomousCommand() {
    return m_auto.getChooser().getSelected();
  }
}
