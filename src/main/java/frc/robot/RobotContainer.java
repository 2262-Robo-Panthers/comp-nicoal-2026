// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;

import frc.robot.subsystems.*;

public class RobotContainer {
  private Shooter m_shooter = new Shooter(3000.0, 10.0, 100.0);

  private CommandXboxController m_operator = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
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

      // Hold X to shoot rapid-fire; press with right bumper to lock

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
