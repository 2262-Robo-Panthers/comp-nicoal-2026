// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;

import frc.robot.subsystems.*;

public class RobotContainer {
  private Drive m_drive = new Drive();

  private Intake m_intake = new Intake(
    /* roller strength */ 1.0
  );

  private Shooter m_shooter = new Shooter(
    Revolutions.per(Minute).of(3000.0),
    Revolutions.per(Minute).of(1000.0),
    Revolutions.per(Minute).of(100.0)
  );

  private Climb m_climb = new Climb(
    Inches.of(12.0) // TODO measure this
  );

  private CommandXboxController m_driver = new CommandXboxController(0);
  private CommandXboxController m_operator = new CommandXboxController(1);

  private static final double kControllerDeadband = 0.07;

  public RobotContainer() {
    configureBindings();
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
          () -> MathUtil.applyDeadband(-m_operator.getLeftY(), kControllerDeadband),
          () -> MathUtil.applyDeadband(-m_operator.getRightY(), kControllerDeadband))
      );

      // Hold X to shoot (controlling the feed); press with right bumper to lock

      m_operator.x().and(m_operator.rightBumper().negate())
        .whileTrue(
          m_shooter.cmd_manualShoot(1.0, () -> -m_operator.getRightY()) // TODO change speed depending on distance
        );

      m_operator.x().and(m_operator.rightBumper())
        .onTrue(
          m_shooter.cmd_manualShoot(1.0, () -> -m_operator.getRightY()) // TODO change speed depending on distance
        );

      // Hold Y to shoot rapid-fire; press with right bumper to lock

      m_operator.y().and(m_operator.rightBumper().negate())
        .whileTrue(
          m_shooter.cmd_autoShoot(1.0) // TODO change speed depending on distance
        );

      m_operator.y().and(m_operator.rightBumper())
        .onTrue(
          m_shooter.cmd_autoShoot(1.0) // TODO change speed depending on distance
        );

      // Press Start to stop the whole shooter

      m_operator.start()
        .onTrue(
          m_shooter.cmd_stop()
        );
    }

    /*** Climb */
    {
      m_climb.setDefaultCommand(
        m_climb.cmd_moveSetpoint(() -> MathUtil.applyDeadband(
          m_operator.getRightTriggerAxis() - m_operator.getLeftTriggerAxis(),
          kControllerDeadband))
      );
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
