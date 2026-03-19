package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class Auto {
  public SendableChooser<Command> m_chooser = new SendableChooser<>();

  public Auto(Command... commands) {
    for (int i = 0; i < commands.length; i++) {
      if (i == 0) {
        m_chooser.setDefaultOption(commands[i].getName(), commands[i]);
      } else {
        m_chooser.addOption(commands[i].getName(), commands[i]);
      }
    }
  }

  public SendableChooser<Command> getChooser() {
    return m_chooser;
  }
}
