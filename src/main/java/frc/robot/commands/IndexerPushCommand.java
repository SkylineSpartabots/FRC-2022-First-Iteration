package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;

//turns on the indexer. TODO: figure out optimal indexer speed.
public class IndexerPushCommand extends CommandBase{
    private final IndexerSubsystem m_subsystem;
    private final Timer m_timer = new Timer();
    private final double m_durationPush = 4;
    private BooleanSupplier executingSupplier;

    public IndexerPushCommand(BooleanSupplier buttonPressed){
        executingSupplier = buttonPressed;
        m_subsystem = IndexerSubsystem.getInstance();
        addRequirements(m_subsystem);
    }

    public IndexerPushCommand(){
        //Create some kind of auto-based executingSupplier for this constructor
        executingSupplier = null;
        m_subsystem = IndexerSubsystem.getInstance();
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
        m_subsystem.setIndexerPowerPercent(Constants.IndexerConstants.indexerSpeedPercent);
    }

    @Override
    public void execute(){
        //implement some sort of encoder in Indexer to make sure it doesn't jam.
    }

    @Override
    public void end(boolean interrupted){
        m_timer.stop();
        m_subsystem.setIndexerPowerPercent(0);
    }

    @Override
    public boolean isFinished(){
        return !executingSupplier.getAsBoolean();
    }
}