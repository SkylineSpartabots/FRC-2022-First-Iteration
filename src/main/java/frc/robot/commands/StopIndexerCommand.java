package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class StopIndexerCommand extends CommandBase {
    private final IndexerSubsystem m_subsystem;
    private boolean isCommandFinished;
    
    public StopIndexerCommand(){
        m_subsystem = IndexerSubsystem.getInstance();
        addRequirements(m_subsystem);
        isCommandFinished = false;
    }

    @Override
    public void initialize(){
        m_subsystem.setIndexerPowerPercent(0);
        isCommandFinished = true;
    }
    public boolean isFinished(){
        return isCommandFinished;
    }
}
