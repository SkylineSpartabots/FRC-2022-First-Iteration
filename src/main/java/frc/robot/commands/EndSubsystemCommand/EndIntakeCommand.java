package frc.robot.commands.EndSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class EndIntakeCommand extends CommandBase {
    private final IntakeSubsystem m_subsystem;

    public EndIntakeCommand() {
        m_subsystem = IntakeSubsystem.getInstance();
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.setIntakePercentPower(0.0);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}
