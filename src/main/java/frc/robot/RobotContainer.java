// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;

import frc.robot.subsystems.*;

public class RobotContainer {
  private Intake m_intake = new Intake(
    /* roller strength */ 1.0
  );

  private Shooter m_shooter = new Shooter(
    Revolutions.per(Minute).of(3000.0),
    Revolutions.per(Minute).of(1000.0),
    Revolutions.per(Minute).of(100.0)
  );

  private CommandXboxController m_operator = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
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
          () -> -m_operator.getLeftY(),
          () -> -m_operator.getRightY())
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
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
