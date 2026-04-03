package frc.robot.auto;

// import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Auto {
  public SendableChooser<Command> m_chooser = new SendableChooser<>();

  public Auto(Command... commands) {
    // m_chooser = AutoBuilder.buildAutoChooser();

    for (int i = 0; i < commands.length; i++) {
      m_chooser.addOption("[Basic] " + commands[i].getName(), commands[i]);
    }

    SmartDashboard.putData("Auto.Choose", m_chooser);
  }

  public SendableChooser<Command> getChooser() {
    return m_chooser;
  }
}
