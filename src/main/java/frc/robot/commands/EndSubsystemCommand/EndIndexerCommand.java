package frc.robot.commands.EndSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class EndIndexerCommand extends CommandBase {
    private final IndexerSubsystem m_subsystem;

    public EndIndexerCommand() {
        m_subsystem = IndexerSubsystem.getInstance();
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.setIndexerPercentPower(0.0);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}
